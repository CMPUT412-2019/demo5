import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from typing import TYPE_CHECKING, Callable, Optional, Set
from smach import State, Sequence
from smach_ros import IntrospectionServer
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from tf.transformations import quaternion_multiply, unit_vector, quaternion_conjugate
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Twist
import random

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
            pose = self.goal()
            if pose is None:
                return 'err'
            print(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)

            orientation = pose.pose.orientation
            orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
            offset = qv_mult(orientation, [0, 0, 1]) * 0.5
            position = Point(pose.pose.position.x + offset[0],
                             pose.pose.position.y + offset[1],
                             pose.pose.position.z + offset[2])

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'odom'
            goal.target_pose.pose.position = position
            goal.target_pose.pose.orientation.w = 1
            self.client.send_goal(goal)
            if self.client.wait_for_result():
                break

        return 'ok'


class MarkerTracker:
    def __init__(self):
        self.ar_tags_subscriber = SubscriberValue('ar_pose_marker', AlvarMarkers)
        self.poses = {}
        self.times = {}
        self.timeout = 2

    def get_pose(self, marker_id):  # type: (int) -> Optional[PoseStamped]
        pose = next((m.pose for m in self.ar_tags_subscriber.value.markers if m.id == marker_id), None)
        time = rospy.get_time()
        if pose is not None:
            self.poses[marker_id] = pose
            self.times[marker_id] = time
            return pose
        elif marker_id in self.poses and time - self.times[marker_id] < self.timeout:
            return self.poses[marker_id]
        else:
            return None

    def get_visible_markers(self):  # type: () -> Set[int]
        markers = self.ar_tags_subscriber.value.markers
        time = rospy.get_time()
        visible_markers = set()
        visible_markers.update(m.id for m in markers)
        for marker_id, marker_time in self.times.items():
            if time - marker_time < self.timeout:
                visible_markers.add(marker_id)
        return visible_markers


class NavigateToMarkerState(State):
    def __init__(self, marker_tracker):  # type: (MarkerTracker) -> None
        State.__init__(self, outcomes=['ok', 'err'], input_keys=['marker_id'])
        self.marker_tracker = marker_tracker

    def execute(self, ud):
        return NavigateToMovingGoalState(lambda: self.marker_tracker.get_pose(ud.marker_id)).execute({})


class FindMarkerState(State):
    def __init__(self, marker_tracker):  # type: (MarkerTracker) -> None
        State.__init__(self, outcomes=['ok', 'err'], input_keys=['marker_id'])
        self.marker_tracker = marker_tracker
        self.twist_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)

    def execute(self, ud):
        rate = rospy.Rate(10)
        while self.marker_tracker.get_pose(ud.marker_id) is None and not rospy.is_shutdown():
            t = Twist()
            t.angular.z = 0.3
            self.twist_publisher.publish(t)
            rate.sleep()
        for _ in range(10):
            self.twist_publisher.publish(Twist())
            rate.sleep()
        return 'ok'


class NavigateToStartState(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'err'])
        self.pose = PoseStamped()
        self.pose.pose.orientation.w = 1.0
        self.pose.header.frame_id = 'odom'

    def execute(self, ud):
        return NavigateToMovingGoalState(lambda: self.pose).execute({})


class SelectMarkerState(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'err'], output_keys=['marker_id'])

    def execute(self, ud):
        ud.marker_id = random.choice([6])  #[3, 6, 9])
        return 'ok'


class SearchForNextMarkerState(State):
    def __init__(self, marker_tracker):  # type: (MarkerTracker) -> None
        State.__init__(self, outcomes=['ok', 'done'], output_keys=['marker_id'])
        self.chosen_markers = set()
        self.marker_tracker = marker_tracker
        self.twist_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)

    def execute(self, ud):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            candidates = self.marker_tracker.get_visible_markers() - self.chosen_markers
            if candidates:
                marker_id = next(iter(candidates))
                self.chosen_markers.add(marker_id)
                ud.marker_id = marker_id
                return 'ok'
            t = Twist()
            t.angular.z = 0.3
            self.twist_publisher.publish(t)
            rate.sleep()
        for _ in range(10):
            self.twist_publisher.publish(Twist())
            rate.sleep()
        return 'ok'



rospy.init_node('docker')
marker_tracker = MarkerTracker()

TARGET = 9

seq = Sequence(outcomes=['ok', 'err', 'done'], connector_outcome='ok')
with seq:
    Sequence.add('SelectMarker', SearchForNextMarkerState(marker_tracker))
    Sequence.add('FindMarker', FindMarkerState(marker_tracker))
    Sequence.add('GoToMarker', NavigateToMarkerState(marker_tracker))
    Sequence.add('GoToStart', NavigateToStartState(), transitions={'ok': 'SelectMarker'})


sis = IntrospectionServer('smach_server', seq, '/SM_ROOT')
sis.start()

seq.execute()
