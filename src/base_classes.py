#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import time
import tfx
import smach
import rospy

from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm

from geometry_msgs.msg import PoseArray, PoseStamped, Point, Polygon, PolygonStamped
from visualization_msgs.msg import Marker

import IPython

SLEEP_TIME = 0

class MasterClass(smach.State):
    """ This is a the base test counterlass for the testing states.

        The excute method simply iterates through all the possible transitions
        from the state. """
    counter = 0

    def __init__(self):

        self.outcomes = None

        self.homePoseRight = None

        self.homePoseLeft = None

    def execute(self, userdata):        
        smach.loginfo(self.__class__.__name__ + " must be subclassed. It does nothing.")
        time.sleep(SLEEP_TIME)
        self.counter += 1
        if self.outcomes is None:
            self.outcomes = self.get_registered_outcomes()
        return self.outcomes[self.counter%len(self.outcomes)]

class Start(MasterClass):
    def __init__(self, davinciArmRight, davinciArmLeft):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.counter = 0

    def execute(self, userdata):
        print "State: Start"
        return 'success'

class IdentifyGraspPoint(MasterClass):
    def __init__(self, davinciArmRight, davinciArmLeft):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.counter = 0
        self.graspPoint = None

    def execute(self, userdata):
        print "State: IdentifyGraspPoint"
        self.counter = MasterClass.counter
        while True:
            rospy.sleep(0.1)
            # Break from loop when grasp point is identified
        userdata.retractionStagingPose = self.graspPoint
        return 'success'

class PlanTrajToGraspPoint(MasterClass):
    def __init__(self, davinciArmRight, davinciArmLeft):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.counter = 0
        self.graspPoint = None

    def execute(self, userdata):
        return 'success'


class MoveToGraspPoint(MasterClass):
    def __init__(self, davinciArmRight, davinciArmLeft):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'], input_keys = ['retractionStagingPose'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.graspPoint = None

    def execute(self, userdata):
        print "State: MoveToRetractionStagingArea"

        # rospy.loginfo('Execute MoveToRetractionStagingArea')
        # raw_input()

        self.davinciArm.setGripperPositionDaVinci(0.3)
        pose = userdata.retractionStagingPose._obj
        self.davinciArm.executeInterpolatedTrajectory(pose)
        return 'success'

class HomePositionRight(MasterClass):
    def __init__(self, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmRight = davinciArmRight
    
    def execute(self, userdata):
        print "State: HomePositionRight"
        self.davinciArmRight.executeInterpolatedTrajectory(self.homePoseRight)

        rospy.loginfo('R - Wait After finishing home pose')
        raw_input()

        return 'success'

class HomePositionLeft(MasterClass):
    def __init__(self, davinciArmLeft):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: HomePositionLeft"
        self.davinciArmLeft.executeInterpolatedTrajectory(self.homePoseLeft)

        rospy.loginfo('L - Wait After finishing home pose')
        raw_input()

        return 'success'

class GraspGak(MasterClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys = ['dropOffPose'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: ReleaseGripper"
        rospy.sleep(2)
        pose = userdata.dropOffPose._obj
        for i in range(8):
            self.davinciArm.setGripperPositionDaVinci(0.10 * i)
            self.davinciArm.goToGripperPose(pose)
        # self.davinciArm.executeInterpolatedTrajectory(self.dropOffPose)
        return 'success'

class MoveToGraspPoint(MasterClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'], input_keys = ['graspPoint'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: MoveToGraspPoint"

        rospy.loginfo('Press enter to execute MoveToGraspPoint')
        raw_input()

        pose = userdata.graspPoint._obj
        self.davinciArm.executeInterpolatedTrajectory(pose)
        return 'success'

class GraspBlock(MasterClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'], input_keys = ['graspPoint'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: GraspBlock"
        pose = userdata.graspPoint._obj
        self.davinciArm.setGripperPositionDaVinci(-0.25)
        self.davinciArm.goToGripperPose(pose)
        # self.davinciArm.executeInterpolatedTrajectory(pose)
        return "success"        

class CheckGrasp(MasterClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['graspPoint'])
        self.davinciArm = davinciArm
        # rospy.Subscriber("/triangle_pose_array", PoseArray, self.pose_array_callback)
        # self.failure_publisher = rospy.Publisher("/failed_grasp_text", Marker)
        self.poses = None
    
    def execute(self, userdata):
        print "State: CheckGrasp"
        return 'success'

    def pose_array_callback(self, msg):
        poses = tfx.pose(msg)
        # sort the poses
        poses.sort(key = lambda p: p.position.x, reverse = True)
        self.poses = poses


class ReleaseGrippersNoBlock(MasterClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: ReleaseGrippersNoBlock"
        return 'success'

class ReturnToRetractionStagingAreaWithBlock(MasterClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys = ['retractionStagingPose'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: ReturnToRetractionStagingAreaWithBlock"
        # self.davinciArm.executeInterpolatedTrajectory(self.retractionStagingPose)

        pose = userdata.retractionStagingPose._obj
        pose.position.x += 0.002
        self.davinciArm.executeInterpolatedTrajectory(pose)
        return 'success'

class MoveToDropOffStagingArea(MasterClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys = ['dropOffStagingPose'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: MoveToDropOffStagingArea"

        pose = userdata.dropOffStagingPose._obj
        self.davinciArm.executeInterpolatedTrajectory(pose)
        return 'success'

class MoveToDropOffPoint(MasterClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys = ['dropOffPose'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: MoveToDropOffPoint"
        pose = userdata.dropOffPose._obj
        self.davinciArm.executeInterpolatedTrajectory(pose)
        return 'success'

class ReleaseGripper(MasterClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys = ['dropOffPose'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: ReleaseGripper"
        rospy.sleep(2)
        pose = userdata.dropOffPose._obj
        for i in range(8):
            self.davinciArm.setGripperPositionDaVinci(0.10 * i)
            self.davinciArm.goToGripperPose(pose)


        # self.davinciArm.executeInterpolatedTrajectory(self.dropOffPose)
        return 'success'

class CheckDropOff(MasterClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['stillLooping','success', 'failure'], input_keys = ['dropOffPose'])
        self.davinciArm = davinciArm
        self.counter = 0

    def execute(self, userdata):
        print "State: CheckDropOff"
        # self.davinciArm.executeInterpolatedTrajectory(self.homePose)
        # self.davinciArm.stop()
        StateTestClass.counter+=1
        return 'stillLooping'

class Abort(MasterClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: Abort"
        return 'success'