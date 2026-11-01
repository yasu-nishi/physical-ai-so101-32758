from types import SimpleNamespace
import pytest
from so101_mcp.servo_diagnostics import AuditedReads,CheckedReads,read_only_instruction,validate_feedback_packet,feedback_phase


def packet(payload, motor=6,error=0):
    row=[255,255,motor,len(payload)+2,error,*payload]
    return row+[~sum(row[2:])&255]


def test_disconnected_feedback_cannot_claim_position_hold():
    assert feedback_phase({},[{'type':'read_error'}])=='feedback_unverified'


def test_restarted_torque_disabled_is_distinct_from_stopped_hold():
    names=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
    values={key:{n:0 for n in names} for key in ('Present_Position','Goal_Position','Torque_Enable','Status')}
    assert feedback_phase(values,[])=='observed_torque_disabled'
    values['Torque_Enable']={n:int(n!='gripper') for n in names}
    assert feedback_phase(values,[])=='stopped_holding'
    assert feedback_phase(values,[{'type':'voltage_guard'}])=='holding_with_fault'
    assert feedback_phase(values,[{'type':'read_error'}])=='feedback_unverified'


def test_normal_voltage_and_position_payloads():
    validate_feedback_packet(packet([48]),1,{6})
    validate_feedback_packet(packet([20,8]),2,{6})


def test_recorded_recovery_fault_identifies_the_motor_without_accepting_the_read():
    from so101_mcp.servo_diagnostics import ServoReportedError
    # Actual position reply from the failed standby recovery, 2026-11-01.
    reply = [255, 255, 6, 4, 1, 105, 10, 129]
    with pytest.raises(ServoReportedError, match='Motor ID 6: Servo error flags in response: 1'):
        validate_feedback_packet(reply, 2, {1, 2, 3, 4, 5, 6})


def test_checksummed_ack_cannot_be_misread_as_voltage():
    ack=packet([])
    assert ack[5]==247  # SDK readRx slicing could expose this checksum as data.
    with pytest.raises(ValueError,match='length'):validate_feedback_packet(ack,1,{6})


@pytest.mark.parametrize('value,expected',[(packet([48],motor=5),'ID'),(packet([48],error=1),'error flags'),(packet([48])[:-1]+[0],'checksum'),(packet([48,0]),'length')])
def test_invalid_feedback_rejected(value,expected):
    with pytest.raises(ValueError,match=expected):validate_feedback_packet(value,1,{6})


@pytest.mark.parametrize('instruction',[3,4,5,6,0x83,0x92])
def test_read_only_audit_blocks_motor_write_instruction(instruction):
    with pytest.raises(ValueError,match='non-read'):read_only_instruction([255,255,1,2,instruction,0])


def test_write_block_happens_before_serial_transmit():
    sent=[];records=[]
    bus=SimpleNamespace(port_handler=SimpleNamespace(writePort=lambda p:sent.append(p)),
                        packet_handler=SimpleNamespace(rxPacket=lambda p:(packet([48]),0)))
    audit=AuditedReads(bus,records.append)
    with pytest.raises(ValueError):bus.port_handler.writePort([255,255,1,2,3,0])
    assert not sent and audit.blocked_non_read_packets==1
    bus.port_handler.writePort([255,255,1,2,2,0])
    assert len(sent)==1 and audit.read_packets==1
    audit.restore()


def test_checked_reader_keeps_controller_write_path_and_rejects_servo_errors():
    sent=[];records=[]
    original=lambda p:(packet([48],error=1),0)
    writer=lambda p:sent.append(p)
    bus=SimpleNamespace(port_handler=SimpleNamespace(writePort=writer),
                        packet_handler=SimpleNamespace(rxPacket=original))
    reader=CheckedReads(bus,records.append)
    assert bus.port_handler.writePort is writer
    reader.context={'bytes':1,'motor_ids':[6]}
    with pytest.raises(ValueError,match='error flags'):bus.packet_handler.rxPacket(None)
    assert reader.servo_error_packets==1 and not sent
    reader.context=None
    assert bus.packet_handler.rxPacket(None)==original(None)
    reader.restore()
    assert bus.packet_handler.rxPacket is original


def test_bad_feedback_is_saved_before_rejection():
    records=[]
    bus=SimpleNamespace(port_handler=SimpleNamespace(writePort=lambda p:None),
                        packet_handler=SimpleNamespace(rxPacket=lambda p:(packet([]),0)))
    audit=AuditedReads(bus,records.append)
    audit.context={'bytes':1,'motor_ids':[6]}
    with pytest.raises(ValueError):bus.packet_handler.rxPacket(None)
    assert audit.invalid_packets==1 and records[0]['packet']==packet([])
    assert 'validation_error' in records[0]
    audit.restore()


def test_valid_packet_with_hardware_error_is_not_classified_as_malformed():
    records=[]
    bus=SimpleNamespace(port_handler=SimpleNamespace(writePort=lambda p:None),
                        packet_handler=SimpleNamespace(rxPacket=lambda p:(packet([37],error=1),0)))
    audit=AuditedReads(bus,records.append);audit.context={'bytes':1,'motor_ids':[6]}
    with pytest.raises(ValueError,match='error flags'):bus.packet_handler.rxPacket(None)
    assert audit.invalid_packets==0 and audit.servo_error_packets==1
    assert records[0]['error_category']=='servo_reported_error'
    audit.restore()


def test_aborted_group_read_clears_host_input_before_next_request(monkeypatch):
    import lerobot.motors.motors_bus as module
    monkeypatch.setattr(module,'get_address',lambda *args:(62,1))
    monkeypatch.setattr('so101_mcp.servo_diagnostics.time.sleep',lambda _:None)
    cleared=[];records=[]
    def failed(*args,**kwargs):raise ValueError('aborted group read')
    bus=SimpleNamespace(port_handler=SimpleNamespace(writePort=lambda p:None,
                            ser=SimpleNamespace(reset_input_buffer=lambda:cleared.append(True))),
                        packet_handler=SimpleNamespace(rxPacket=lambda p:(packet([48]),0)),
                        motors={'gripper':SimpleNamespace(id=6,model='test')},model_ctrl_table={},sync_read=failed)
    audit=AuditedReads(bus,records.append)
    with pytest.raises(ValueError,match='aborted'):audit.read('Present_Voltage')
    assert cleared==[True] and records[-1]['kind']=='read_failure_input_resync'
    assert audit.context is None and audit.read_packets==0
    audit.restore()
