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

        self.homePoseLeft = tfx.pose([-0.04907726751924995, 0.027288735500984575, -0.1211606501908539],
            (0.9835887535507493, -0.09932464655036198, -0.14884734393715604, 0.023070472014753367))
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
        smach.State.__init__(self, outcomes = ['success', 'failure'], output_keys = ['graspPoint', 'counter', 'maxDebris'])
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
        if not self.graspPointFound:
            graspCount = 0
            for graspPose in msg.poses:
                graspPose = tfx.pose(graspPose)
                graspPose.stamp = msg.header.stamp
                graspPose.frame = msg.header.frame_id
                self.allGraspPoints[graspCount] = graspPose
                graspCount += 1
            self.graspPointFound = True

    def execute(self, userdata):
        print "State: IdentifyGraspPoint"
        if self.counterDebris == -1:        
            while True:
                rospy.sleep(0.1)
                if self.graspPointFound:
                    break
            self.counterDebris += 1
        self.graspPoint = self.allGraspPoints[self.counterDebris]
        self.graspPoint.position.y += -0.014
        self.graspPoint.position.x += -0.009
        self.graspPoint.position.z += -0.015
        userdata.graspPoint = self.graspPoint
        userdata.counter = self.counterDebris
        userdata.maxDebris = len(self.allGraspPoints)
        # rospy.loginfo('Pause')
        # raw_input()
        self.publisher.publish(self.graspPoint.msg.PoseStamped())
        print 'XXX_ Count', self.counterDebris
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
        currPoseRight.position.z += 0.04

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
        currPoseRight.position.y += 0.0085
        currPoseRight.position.z += -0.025
        currPoseRight.position.x += 0.004
        currPoseRight.stamp = None
        cutPointCurr = tfx.convertToFrame(currPoseRight, '/one_remote_center_link')
        self.cut_point_pub.publish(cutPointCurr.msg.PoseStamped())

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
        # rospy.loginfo('Enter to MoveToPreCutPoint')
        # raw_input()

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

        rospy.loginfo('Done with Cleaning')
        raw_input()

        maxDebris = userdata.maxDebris
        maxDebris -= 1
        counter = userdata.counter
        if counter < maxDebris:
            return 'loop'
        return 'success'

class Abort(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: Abort"
        return 'success'