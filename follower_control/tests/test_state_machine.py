import pytest
from follower_control.state_machine import ControlFSM


def test_initial_state_tracking():
    assert ControlFSM().state == 'TRACKING'


def test_lost_then_reacquired():
    fsm = ControlFSM()
    fsm.lost()
    assert fsm.state == 'SEARCHING'
    fsm.reacquired()
    assert fsm.state == 'TRACKING'


def test_search_failed_ends():
    fsm = ControlFSM()
    fsm.lost()
    fsm.search_failed()
    assert fsm.state == 'ENDED'


def test_restart_from_ended():
    fsm = ControlFSM()
    fsm.lost()
    fsm.search_failed()
    fsm.restart()
    assert fsm.state == 'TRACKING'


def test_invalid_transition_raises():
    from transitions import MachineError
    fsm = ControlFSM()
    with pytest.raises(MachineError):
        fsm.reacquired()          # not valid from TRACKING
