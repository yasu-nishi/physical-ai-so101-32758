"""Read-only wire audit for Feetech feedback. No torque or motion commands.

The installed SDK accepts some checksummed packets with a payload length that
does not match the read request. This wrapper rejects those packets and records
them; finding that software gap does not prove it caused a hardware incident.
"""
from datetime import datetime, timezone
import time


class FeedbackProtocolError(ValueError):
    pass


class ServoReportedError(ValueError):
    pass


def feedback_phase(values, errors):
    """Never report a holding pose from missing or failed feedback."""
    required=('Present_Position','Goal_Position','Torque_Enable','Status')
    if any(k not in values for k in required) or any(e['type']=='read_error' for e in errors):
        return 'feedback_unverified'
    torque=values['Torque_Enable']
    if not torque:
        return 'feedback_unverified'
    if all(v==0 for v in torque.values()):
        return 'observed_torque_disabled'
    if len(torque)==6 and all(v==int(n!='gripper') for n,v in torque.items()):
        return 'stopped_holding' if not errors else 'holding_with_fault'
    return 'observed_mixed_torque'


def validate_feedback_packet(packet, payload_bytes, motor_ids):
    if len(packet) != payload_bytes+6 or packet[:2] != [255,255] or packet[3] != payload_bytes+2:
        raise FeedbackProtocolError('Feedback packet length/header does not match read request')
    if packet[2] not in motor_ids:
        raise FeedbackProtocolError('Unexpected motor ID in feedback packet')
    if sum(packet[2:]) & 255 != 255:
        raise FeedbackProtocolError('Feedback packet checksum mismatch')
    if packet[4]:
        raise ServoReportedError(f'Motor ID {packet[2]}: Servo error flags in response: {packet[4]}')


def read_only_instruction(packet):
    values=list(packet)
    # PING, READ and SYNC_READ only. No WRITE, REG_WRITE, ACTION or SYNC_WRITE.
    if len(values)<6 or values[:2]!=[255,255] or values[4] not in (1,2,0x82):
        raise ValueError('Read-only diagnostic blocked a non-read serial instruction')
    return values[4]


class CheckedReads:
    """Validate feedback for a single serial owner; does not grant or block writes.

    A bounded motion controller may use this reader while retaining its own
    write allowlist. Use AuditedReads for a physically read-only diagnostic.
    """
    def __init__(self,bus,record):
        self.bus,self.record=bus,record
        self.context=None
        self.read_packets=0
        self.blocked_non_read_packets=0
        self.rx_packets=0
        self.invalid_packets=0
        self.servo_error_packets=0
        self.original_rx=bus.packet_handler.rxPacket
        bus.packet_handler.rxPacket=self.receive

    def receive(self,port):
        packet,result=self.original_rx(port)
        self.rx_packets+=1
        entry={'kind':'rx','context':self.context,'result':result,'packet':list(packet)}
        error=None
        if self.context and result==0:
            try:validate_feedback_packet(list(packet),self.context['bytes'],self.context['motor_ids'])
            except ValueError as e:
                if isinstance(e,ServoReportedError):
                    self.servo_error_packets+=1;entry['error_category']='servo_reported_error'
                else:
                    self.invalid_packets+=1;entry['error_category']='protocol_error'
                entry['validation_error']=str(e);error=e
        self.record(entry)
        if error:raise error
        return packet,result

    def read(self,register,motor=None,*,normalize=False):
        from lerobot.motors.motors_bus import get_address
        names=[motor] if motor else list(self.bus.motors)
        addresses=[get_address(self.bus.model_ctrl_table,self.bus.motors[n].model,register) for n in names]
        if len(set(addresses))!=1:raise ValueError('Mixed register definitions')
        addr,length=addresses[0]
        self.context={'register':register,'address':addr,'bytes':length,
                      'motor_ids':[self.bus.motors[n].id for n in names]}
        try:
            if motor:return self.bus.read(register,motor,normalize=normalize)
            return self.bus.sync_read(register,normalize=normalize)
        except Exception:
            # An aborted group read can leave replies from later motors queued.
            # SDK clearPort() calls serial.flush(), which does not clear input.
            # Save the failure, wait for that bounded request to drain, and clear
            # host input before the NEXT sample. Never retry a failed motion.
            time.sleep(.06)
            serial=self.bus.port_handler.ser
            serial.reset_input_buffer()
            self.record({'kind':'read_failure_input_resync','context':self.context,
                         'host_input_buffer_cleared':True,'motor_register_writes':0})
            raise
        finally:self.context=None

    def restore(self):
        self.bus.packet_handler.rxPacket=self.original_rx


class AuditedReads(CheckedReads):
    """Checked feedback plus a serial-level block on every non-read command."""
    def __init__(self,bus,record):
        super().__init__(bus,record)
        self.original_write=bus.port_handler.writePort
        bus.port_handler.writePort=self.transmit

    def transmit(self,packet):
        try:instruction=read_only_instruction(packet)
        except ValueError:
            self.blocked_non_read_packets+=1
            raise
        self.read_packets+=1
        self.record({'kind':'tx_read','context':self.context,'instruction':instruction,'packet':list(packet)})
        return self.original_write(packet)

    def restore(self):
        self.bus.port_handler.writePort=self.original_write
        super().restore()


def timestamped(row):
    return {'time_utc':datetime.now(timezone.utc).isoformat(),'monotonic_s':time.monotonic(),**row}
