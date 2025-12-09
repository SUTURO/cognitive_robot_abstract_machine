from enum import Enum

from pycram.datastructures.pose import PoseStamped


class NavigatePosition(Enum):
    START_DOOR = PoseStamped.from_list(
        position=[0.25574201345443726, 0.009331032633781433, 0.0],
        orientation=[0.0, 0.0, 0.10044125411469684, 0.9949429905637142],
    )
    IN_FRONT_DISHWASHER = PoseStamped.from_list(
        position=[2.322282075881958, -1.2466719150543213, 0.0],
        orientation=[0.0, 0.0, -0.6304255522851391, 0.7762497169248935],
    )
    SOFA_FACING = PoseStamped.from_list(
        position=[3.325042486190796, 2.6914186477661133, 0.0],
        orientation=[0.0, 0.0, -0.6548372471824481, 0.75576992511115],
    )
    SOFA_BACK = PoseStamped.from_list(
        position=[3.390072822570801, 2.5000317096710205, 0.0],
        orientation=[0.0, 0.0, 0.7573494640106083, 0.6530097927005722],
    )
    CABINET = PoseStamped.from_list(
        position=[3.8683114051818848, 5.459158897399902, 0.0],
        orientation=[0.0, 0.0, 0.04904329912700753, 0.9987966533838301],
    )
    GROCERY_TABLE = PoseStamped.from_list(
        position=[3.276226282119751, 5.599549770355225, 0.0],
        orientation=[0.0, 0.0, 0.978813109688997, 0.20475569906831742],
    )
