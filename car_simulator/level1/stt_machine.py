from transitions import Machine

class SttMachine:
    #initialize
    def __init__(self, states, transitions, initial):
          
        # states = ['idle','follow_path','brake']
        # transitions = [
        #     {'trigger': 'start', 'source': 'idle', 'dest': 'follow_path'},
        #     {'trigger': 'pause', 'source': 'follow_path', 'dest': 'brake'},
        #     {'trigger': 'abort', 'source': 'follow_path', 'dest': 'idle'},
        #     {'trigger': 'finish', 'source': 'follow_path', 'dest': 'idle'}
        # ]

        self.machine = Machine(states=states, transitions=transitions, initial='idle')

    def get_state(self):
        print(f"state: {self.machine.state}")
        return self.machine.state