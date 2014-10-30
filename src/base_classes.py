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

from geometry_msgs.msg import PoseStamped, PoseArray, PointStamped

import IPython

SLEEP_TIME = 0

class MasterClass(smach.State):
    """ This is a the base test counterlass for the testing states.

        The excute method simply iterates through all the possible transitions
        from the state. """

    def __init__(self):

        self.outcomes = None

        self.homePoseLeft = tfx.pose([-0.05009142015689537, 0.03540081898889297, -0.14583058800170023],
            (-0.01797605558439532, -0.9838454461982115, 0.03728902700169569, 0.17416810237794897))

        self.homePoseRight = tfx.pose([0.061241857051286236, 0.014307808069346816, -0.10446866837544996],
            (-0.9689889616996428, 0.1939060552483166, -0.1474787309756946, 0.04136251626876463))

    def execute(self, userdata):        
        smach.loginfo(self.__class__.__name__ + " must be subclassed. It does nothing.")
        time.sleep(SLEEP_TIME)
        self.counter += 1
        if self.outcomes is None:
            self.outcomes = self.get_registered_outcomes()
        return self.outcomes[self.counter%len(self.outcomes)]

class Start(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: Start"
        return 'success'

class IdentifyGraspPoint(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure', 'complete'], output_keys = ['graspPoint', 'counter', 'maxDebris'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.graspPoint = None
        self.allGraspPoints = {}
        self.black_dot_point = None
        self.graspPointFound = False
        self.counterDebris = -1

        rospy.Subscriber("/gak/grasp_poses", PoseArray, self.grasp_point_callback)
        self.publisher = rospy.Publisher("/gak/curr_grasp_pose", PoseStamped)

    def grasp_point_callback(self, msg):
        self.allGraspPoints = {}
        if not self.graspPointFound:
            graspCount = 0
            for graspPose in msg.poses:
                graspPose = tfx.pose(graspPose)
                graspPose.stamp = msg.header.stamp
                graspPose.frame = msg.header.frame_id
                self.allGraspPoints[graspCount] = graspPose
                graspCount += 1

            self.graspPoint = tfx.pose(self.get_grasp_point(self.allGraspPoints.values()), copy = True)
            self.graspPointFound = True

    def get_grasp_point(self, allGraspPoses):
        ycoord_to_pose_dict = {}
        for pose in allGraspPoses:
            ycoord_to_pose_dict[pose.position.y] = pose
        min_y = min(ycoord_to_pose_dict.keys())
        return ycoord_to_pose_dict[min_y]

    def execute(self, userdata):
        print "State: IdentifyGraspPoint"
        i = 0
        self.graspPointFound = False
        while True:
            rospy.sleep(0.1)
            if self.graspPointFound:
                break
            if i == 30:
                return 'complete'
            i += 1
        self.counterDebris += 1
        self.graspPoint.position.y += -0.015
        self.graspPoint.position.x += -0.009
        self.graspPoint.position.z += -0.015

        print self.graspPoint

        userdata.graspPoint = self.graspPoint
        userdata.counter = self.counterDebris
        userdata.maxDebris = len(self.allGraspPoints)
        self.publisher.publish(self.graspPoint.msg.PoseStamped())
        self.counterDebris += 1
        return 'success'

class PlanTrajToGraspPointRight(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.graspPoint = None

    def execute(self, userdata):
        print "State: PlanTrajToGraspPointRight"
        return 'success'

class MoveToPreGraspPoint(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'], input_keys = ['graspPoint'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.graspPoint = None

    def execute(self, userdata):
        print "State: MoveToPreGraspPoint"

        pose = tfx.pose(userdata.graspPoint._obj, copy = True)
        print "Grasp Point", pose

        pose.position.z += 0.012
        print "Pre Grasp Point", pose

        # rospy.loginfo('Execute MoveToPreGraspPoint')
        # raw_input()

        self.davinciArmRight.executeInterpolatedTrajectory(pose)
        return 'success'

class MoveToGraspPoint(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'], input_keys = ['graspPoint'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.graspPoint = None

    def execute(self, userdata):
        print "State: MoveToGraspPoint"

        graspPoint = userdata.graspPoint._obj
        # rospy.loginfo('Execute MoveToGraspPoint')
        # raw_input()

        self.davinciArmRight.setGripperPositionDaVinci(1)
        self.davinciArmRight.executeInterpolatedTrajectory(graspPoint)

        return 'success'

class HomePositionRight(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
    
    def execute(self, userdata):
        print "State: HomePositionRight"

        # rospy.loginfo('Execute HomePositionRight')
        # raw_input()

        return 'success'

class HomePositionLeft(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: HomePositionLeft"

        # rospy.loginfo('Execute HomePositionLeft')
        # raw_input()

        return 'success'

class GraspGak(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success'])
        self.davinciArmLeft = davinciArmLeft
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        print "State: GraspGak"

        # rospy.loginfo('Enter to Grasp Gak')
        # raw_input()

        self.davinciArmRight.setGripperPositionDaVinci(-1)
        currPoseRight = self.davinciArmRight.getGripperPose()

        self.davinciArmRight.executeInterpolatedTrajectory(currPoseRight)
        return 'success'

class RetractGak(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: Retract Gak"
        # rospy.loginfo('Enter to Retract Gak')
        # raw_input()
        currPoseRight = self.davinciArmRight.getGripperPose()
        rospy.sleep(1)
        currPoseRight.position.z += 0.03

        self.davinciArmRight.goToGripperPose(currPoseRight, speed=0.005)
        return 'success'

class CheckGrasp(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: Check Grasp "
        return 'success'

class ReleaseGrippersNoGak(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: ReleaseGrippersNoGak"

        # rospy.loginfo('Enter to Release Grippers')
        # raw_input()

        return 'success'

class IdentifyCutPoint(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'], output_keys = ['cutPoint'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.cut_point_pub = rospy.Publisher('/gak/cut_point_pose', PoseStamped)

    def execute(self, userdata):
        print "State: IdentifyCutPoint"

        # rospy.loginfo('Enter to Identity Cut Point')
        # raw_input()

        currPoseRight = self.davinciArmRight.getGripperPose()
        currPoseRight = currPoseRight.as_tf()*tfx.pose(tfx.tb_angles(180,0,0)).as_tf()*tfx.pose(tfx.tb_angles(0,-75,0))
        currPoseRight.position.y += 0.009
        currPoseRight.position.z += -0.03
        currPoseRight.position.x += 0.004
        currPoseRight = currPoseRight.as_tf()*tfx.pose(tfx.tb_angles(180,0,0))
        currPoseRight.stamp = None
        cutPointCurr = tfx.convertToFrame(currPoseRight, '/one_remote_center_link')
        self.cut_point_pub.publish(cutPointCurr.msg.PoseStamped())
        # rospy.loginfo('Check cut point')
        # raw_input()

        userdata.cutPoint = cutPointCurr

        return 'success'

class PlanTrajToPreCutPointLeft(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: PlanTrajToPreCutPointLeft"
        return 'success'

class MoveToPreCutPoint(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys = ['cutPoint'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: MoveToPreCutPoint"
        preCutPoint = tfx.pose(userdata.cutPoint._obj, copy = True)

        preCutPoint = tfx.convertToFrame(preCutPoint, '/two_remote_center_link')
        preCutPoint.position.x += 0.03
        preCutPoint = tfx.convertToFrame(preCutPoint, '/one_remote_center_link')

        print "Left Arm precut", preCutPoint

        self.davinciArmLeft.setGripperPositionDaVinci(1)
        self.davinciArmLeft.executeInterpolatedTrajectory(preCutPoint)
        return 'success'

class CuttingAction(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success'], input_keys = ['cutPoint'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: CuttingAction"
        cutPoint = tfx.pose(userdata.cutPoint._obj, copy = True)
        print "Cut Point: ", cutPoint
        # rospy.loginfo('Enter to MoveToCutPoint')
        # raw_input()

        self.davinciArmLeft.executeInterpolatedTrajectory(cutPoint)
        self.davinciArmLeft.setGripperPositionDaVinci(-2.2)
        self.davinciArmLeft.executeInterpolatedTrajectory(cutPoint)
        return 'success'

class CheckCut(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: CheckCut"
        return 'success'

class Cleaning(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'loop'], input_keys = ['counter', 'maxDebris'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: Clean"

        # rospy.loginfo('Enter to Clean')
        # raw_input()
        self.davinciArmRight.goToGripperPose(self.homePoseRight)
        self.davinciArmRight.setGripperPositionDaVinci(1)
        self.davinciArmRight.goToGripperPose(self.homePoseRight)
        self.davinciArmLeft.goToGripperPose(self.homePoseLeft)
        self.davinciArmLeft.setGripperPositionDaVinci(1)
        self.davinciArmLeft.goToGripperPose(self.homePoseLeft)

        rospy.sleep(3)
        # rospy.loginfo('Done with Cleaning')
        # raw_input()

        # maxDebris = userdata.maxDebris
        # maxDebris -= 1
        # counter = userdata.counter
        # if counter < maxDebris:
        #     return 'loop'
        return 'loop'

class Abort(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: Abort"
        return 'success'