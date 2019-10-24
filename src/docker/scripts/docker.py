import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from typing import TYPE_CHECKING, Callable
from smach import State, Sequence
from smach_ros import IntrospectionServer
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from tf.transformations import quaternion_multiply, unit_vector, quaternion_conjugate
import numpy as np
from geometry_msgs.msg import Point

if TYPE_CHECKING:
    from geometry_msgs.msg import PoseStamped


from util import SubscriberValue


def qv_mult(q1, v1):
    # https: // answers.ros.org / question / 196149 / how - to - rotate - vector - by - quaternion - in -python /
    v1 = unit_vector(v1)
    q2 = list(v1) + [0.0]
    return quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))[:3]


class NavigateToMovingGoalState(State):
    def __init__(self, goal):  # type: (Callable[[], PoseStamped]) -> None
        State.__init__(self, outcomes=['ok', 'err'])
        self.goal = goal
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, ud):
        self.client.wait_for_server()
        while True:
            print(self.goal().pose.position.x, self.goal().pose.position.y, self.goal().pose.position.z)

            orientation = self.goal().pose.orientation
            orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            offset = qv_mult(orientation, [0, 0, 1]) * 0.5
            position = Point(self.goal().pose.position.x + offset[0],
                             self.goal().pose.position.y + offset[1],
                             self.goal().pose.position.z + offset[2])

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.pose.position = position
            goal.target_pose.pose.orientation.w = 1
            self.client.send_goal(goal)
            if self.client.wait_for_result():
                break

        return 'ok'


TARGET = 9

rospy.init_node('docker')
ar_tags_subscriber = SubscriberValue('ar_pose_marker', AlvarMarkers)

target = None
while target is None:
    target = next((m.pose for m in ar_tags_subscriber.value.markers if m.id == TARGET), target)  # type: PoseStamped


seq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
with seq:
    Sequence.add('GetWaypoints', NavigateToMovingGoalState(lambda: next((m.pose for m in ar_tags_subscriber.value.markers if m.id == TARGET), target)))


sis = IntrospectionServer('smach_server', seq, '/SM_ROOT')
sis.start()

seq.execute()