from core.action import Action

class ActivateAutonomousMode(Action):
    def __init__(self, condition=None, callback=None):
        super().__init__(one_shot=True, condition=condition, callback=callback)

    def _do(self, actor, client_node, global_state):
        client_node.send_engage_cmd()