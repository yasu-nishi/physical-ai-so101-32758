import pytest

from so101_mcp.joints import ARM
from so101_mcp.servo_monitor import ServoMonitor


def monitor_fixture(tmp_path):
    names = (*ARM, 'gripper')
    defaults = {'Min_Voltage_Limit': 40, 'Max_Voltage_Limit': 84, 'Protection_Current': 500,
                'Present_Position': 2981, 'Goal_Position': 2958, 'Goal_Position_2': 2958,
                'Present_Velocity': 0, 'Present_Load': 192, 'Present_Current': 11,
                'Present_Voltage': 48, 'Present_Temperature': 35, 'Status': 0, 'Torque_Enable': 1}
    values = {key: dict.fromkeys(names, value) for key, value in defaults.items()}
    reads = []
    def read(key):
        reads.append(key)
        return dict(values[key])
    monitor = ServoMonitor(None, tmp_path, reader=read)
    return monitor, values, reads


def test_monitor_records_actual_feedback_without_writes(tmp_path):
    monitor, values, reads = monitor_fixture(tmp_path)
    result = monitor.sample(require_torque=True)
    assert result['values_raw']['Present_Position']['elbow_flex'] == 2981
    assert not result['issues']
    count = len(reads)
    monitor.sample(require_torque=True)
    assert len(reads) == count
    monitor.sample(force=True)
    assert len(reads) > count


@pytest.mark.parametrize('register,value', [('Status',1), ('Torque_Enable',0),
    ('Present_Voltage',41), ('Present_Temperature',56), ('Present_Current',251), ('Present_Load',501)])
def test_hardware_fault_fails_locally_before_next_command(tmp_path, register, value):
    monitor, values, _ = monitor_fixture(tmp_path)
    values[register]['elbow_flex'] = value
    with pytest.raises(RuntimeError): monitor.sample(require_torque=True)
    assert (tmp_path/'servo_feedback.local.jsonl').exists()


def test_failed_or_missing_feedback_is_not_invented(tmp_path):
    monitor, values, _ = monitor_fixture(tmp_path)
    del values['Status']['elbow_flex']
    with pytest.raises(ValueError, match='Incomplete'): monitor.sample(require_torque=True)



@pytest.mark.parametrize('flags',[1,32])
def test_servo_fault_is_not_hidden_by_a_later_successful_read(tmp_path,flags):
    from so101_mcp.servo_diagnostics import ServoReportedError
    m,values,_=monitor_fixture(tmp_path)
    calls=[]
    def transient(key,*a,**k):
        calls.append(key)
        if len(calls)==1:raise ServoReportedError(f'Servo error flags in response: {flags}')
        return values[key]
    m._raw_read=transient
    with pytest.raises(ServoReportedError):m.sample(require_torque=True,force=True)
    assert len(calls)==1
