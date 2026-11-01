from scripts.compare_servo_reads import fault_register_report

import pytest


def test_isolated_gripper_diagnostic_uses_sdk_read_for_id6_and_blocks_writes(monkeypatch):
    from scripts.compare_servo_reads import make_diagnostic_bus
    from so101_mcp.servo_diagnostics import AuditedReads

    # Use the installed LeRobot and Feetech packet encoder, with a fake wire.
    # No other servo is present and no OS serial device is opened.
    bus = make_diagnostic_bus('/not/a/serial/device', [6])
    sent, records = [], []
    port = bus.port_handler
    port.is_open = True
    monkeypatch.setattr(port, 'clearPort', lambda: None)
    monkeypatch.setattr(port, 'setPacketTimeout', lambda n: None)
    def transmit(packet):
        sent.append(list(packet))
        return len(packet)
    monkeypatch.setattr(port, 'writePort', transmit)
    def reply(port):
        port.is_using = False
        return [255, 255, 6, 3, 0, 48, 198], 0
    monkeypatch.setattr(bus.packet_handler, 'rxPacket', reply)
    audit = AuditedReads(bus, records.append)
    try:
        assert audit.read('Present_Voltage', 'gripper') == 48
        assert sent == [[255, 255, 6, 4, 2, 62, 1, 180]]
        with pytest.raises(ValueError, match='non-read'):
            bus.write('Torque_Enable', 'gripper', 1, normalize=False)
        assert len(sent) == 1
        assert audit.blocked_non_read_packets == 1
    finally:
        audit.restore()
        port.is_open = False


@pytest.mark.parametrize('ids', [[], [6, 6], [0], [7]])
def test_invalid_diagnostic_motor_selection_rejected_without_serial_open(ids):
    from scripts.compare_servo_reads import make_diagnostic_bus
    with pytest.raises(ValueError, match='unique motor IDs'):
        make_diagnostic_bus('/not/a/serial/device', ids)


def test_rejected_low_voltage_packet_remains_visible_in_diagnostic_summary():
    # Captured after the operator pressed the connector, with all torque OFF.
    row = {'error_category': 'servo_reported_error',
           'context': {'register': 'Present_Voltage', 'motor_ids': [6]},
           'packet': [255, 255, 6, 3, 1, 38, 207]}
    report = fault_register_report(row)
    assert report['motor_id'] == 6 and report['reported_raw'] == 38
    assert report['error_flags'] == 1
    assert report['accepted_control_feedback'] is False
    # A corrupt reply cannot be used as evidence of an actual reported value.
    row['packet'][-1] = 0
    assert fault_register_report(row) is None


def test_validated_error_status_is_not_counted_as_a_normal_status():
    row = {'error_category': 'servo_reported_error',
           'context': {'register': 'Status', 'motor_ids': [5]},
           'packet': [255, 255, 5, 3, 1, 1, 245]}
    report = fault_register_report(row)
    assert report['register'] == 'Status' and report['reported_raw'] == 1
    assert report['accepted_control_feedback'] is False
