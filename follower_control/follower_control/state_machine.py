from transitions import Machine


class ControlFSM:
    """Minimal follow-control FSM: TRACKING <-> SEARCHING -> ENDED."""

    states = ['TRACKING', 'SEARCHING', 'ENDED']

    def __init__(self):
        self.machine = Machine(model=self, states=self.states,
                               initial='TRACKING', auto_transitions=False,
                               ignore_invalid_triggers=False)
        self.machine.add_transition('lost', 'TRACKING', 'SEARCHING')
        self.machine.add_transition('reacquired', 'SEARCHING', 'TRACKING')
        self.machine.add_transition('search_failed', 'SEARCHING', 'ENDED')
        self.machine.add_transition('restart', 'ENDED', 'TRACKING')
