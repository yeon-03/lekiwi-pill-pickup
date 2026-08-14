from lekiwi_pill_pickup.pick_state_machine import (
    Action, ActionType, Observation, PickState, tick,
)


def test_searching_transitions_to_approaching_when_detected():
    state, action = tick(
        PickState.SEARCHING, Observation(detected=True),
        elapsed_in_state_sec=1.0, search_timeout_sec=8.0)
    assert state == PickState.APPROACHING
    assert action.type == ActionType.NONE


def test_searching_sweeps_when_not_detected_and_not_timed_out():
    state, action = tick(
        PickState.SEARCHING, Observation(detected=False),
        elapsed_in_state_sec=1.0, search_timeout_sec=8.0)
    assert state == PickState.SEARCHING
    assert action.type == ActionType.SWEEP


def test_searching_fails_on_timeout():
    state, action = tick(
        PickState.SEARCHING, Observation(detected=False),
        elapsed_in_state_sec=8.0, search_timeout_sec=8.0)
    assert state == PickState.FAILED


def test_approaching_nudges_when_not_aligned():
    state, action = tick(
        PickState.APPROACHING, Observation(detected=True, aligned=False),
        elapsed_in_state_sec=0.5, search_timeout_sec=8.0)
    assert state == PickState.APPROACHING
    assert action.type == ActionType.NUDGE


def test_approaching_transitions_to_grasping_when_aligned():
    state, action = tick(
        PickState.APPROACHING, Observation(detected=True, aligned=True),
        elapsed_in_state_sec=0.5, search_timeout_sec=8.0)
    assert state == PickState.GRASPING
    assert action.type == ActionType.DESCEND_AND_GRIP


def test_approaching_returns_to_searching_if_target_lost():
    state, action = tick(
        PickState.APPROACHING, Observation(detected=False),
        elapsed_in_state_sec=0.5, search_timeout_sec=8.0)
    assert state == PickState.SEARCHING


def test_grasping_succeeds_when_grasped():
    state, action = tick(
        PickState.GRASPING, Observation(detected=True, grasped=True),
        elapsed_in_state_sec=0.1, search_timeout_sec=8.0)
    assert state == PickState.LIFTING
    assert action.type == ActionType.LIFT


def test_grasping_fails_when_not_grasped():
    state, action = tick(
        PickState.GRASPING, Observation(detected=True, grasped=False),
        elapsed_in_state_sec=0.1, search_timeout_sec=8.0)
    assert state == PickState.FAILED


def test_lifting_transitions_to_succeeded():
    state, action = tick(
        PickState.LIFTING, Observation(detected=True),
        elapsed_in_state_sec=0.1, search_timeout_sec=8.0)
    assert state == PickState.SUCCEEDED


def test_terminal_states_stay_terminal_and_take_no_action():
    for terminal in (PickState.SUCCEEDED, PickState.FAILED):
        state, action = tick(
            terminal, Observation(detected=True),
            elapsed_in_state_sec=0.1, search_timeout_sec=8.0)
        assert state == terminal
        assert action.type == ActionType.NONE
