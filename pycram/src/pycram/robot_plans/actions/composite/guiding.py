

'''
 If there would be such class it would need parameters for:
    :param = goals: goals that need to be visited
    :param = finalGoal: end goal
'''
@has_parameters
@dataclass
class GuideToAction(ActionDescription):
    def execute(self) -> None:
        Exception("Not implemented yet")

