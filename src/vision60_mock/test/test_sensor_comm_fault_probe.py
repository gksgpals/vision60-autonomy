from vision60_mock.sensor_comm_fault_probe import fault_is_confirmed


def test_delay_requires_a_forwarded_message_older_than_timeout():
    assert fault_is_confirmed('delay', True, True, 0.8, 1.0)
    assert not fault_is_confirmed('delay', True, True, 0.4, 1.0)


def test_drop_requires_raw_input_and_no_forwarded_message():
    assert fault_is_confirmed('drop', True, False, 0.0, 2.0)
    assert not fault_is_confirmed('drop', True, True, 0.0, 2.0)
