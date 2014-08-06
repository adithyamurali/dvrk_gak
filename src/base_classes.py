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

from geometry_msgs.msg import PoseStamped, PointStamped

import IPython

SLEEP_TIME = 0

class MasterClass(smach.State):
    """ This is a the base test counterlass for the testing states.

        The excute method simply iterates through all the possible transitions
        from the state. """
    counter = 0

    def __init__(self):

        self.outcomes = None

        self.homePoseRight = tfx.pose([0.08110923304266986, 0.019082072847487756, -0.07564648655601992],
            (-0.7296703092609553, 0.5879730580371108, -0.28914218075416975, 0.19561626239715652))

        self.homePoseLeft = None

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
        smach.State.__init__(self, outcomes = ['success', 'failure'], output_keys = ['graspPoint'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.graspPoint = None
        self.black_dot_point = None
        self.counter = 0

        rospy.Subscriber("/gak/black_dot_point", PointStamped, self.black_dot_point_callback)

        self.grasp_point_pub = rospy.Publisher("/gak/grasp_point", PoseStamped)

    def black_dot_point_callback(self, msg):
        if self.counter == 0:
            self.black_dot_point = tfx.point(msg)
            self.counter += 1
    
    def execute(self, userdata):
        print "State: IdentifyGraspPoint"
        while True:
            rospy.sleep(0.1)
            if self.black_dot_point is not None:
                break
        pt = tfx.convertToFrame(self.black_dot_point, '/two_remote_center_link')
        pose = tfx.pose(pt)
        self.graspPoint = pose.as_tf()*tfx.pose(tfx.tb_angles(180,0,0)).as_tf()*tfx.pose(tfx.tb_angles(0,0,-90))
        self.grasp_point_pub.publish(graspPoint.msg.PoseStamped())

        userdata.graspPoint = self.graspPoint
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

        rospy.loginfo('Execute MoveToPreGraspPoint')
        raw_input()

        # self.davinciArmRight.setGripperPositionDaVinci(0.3)
        pose = userdata.graspPoint._obj
        # pose.position.
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

        rospy.loginfo('Execute MoveToGraspPoint')
        raw_input()

        self.davinciArmRight.setGripperPositionDaVinci(0.3)
        pose = userdata.graspPoint._obj
        self.davinciArmRight.executeInterpolatedTrajectory(pose)

        return 'success'

class HomePositionRight(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
    
    def execute(self, userdata):
        print "State: HomePositionRight"

        rospy.loginfo('Execute HomePositionRight')
        raw_input()

        self.davinciArmRight.executeInterpolatedTrajectory(self.homePoseRight)

        return 'success'

class HomePositionLeft(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: HomePositionLeft"

        rospy.loginfo('Execute HomePositionLeft')
        raw_input()

        # self.davinciArmLeft.executeInterpolatedTrajectory(self.homePoseLeft)

        return 'success'

class GraspGak(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success'])
        self.davinciArmLeft = davinciArmLeft
        self.davinciArmRight = davinciArmRight

    def execute(self, userdata):
        print "State: GraspGak"

        rospy.loginfo('Enter to Grasp Gak')
        raw_input()
        self.davinciArmRight.setGripperPositionDaVinci(-0.1)
        currPoseRight = self.davinciArmRight.getGripperPose()
        self.davinciArmRight.goToGripperPose(currPoseRight)
        rospy.sleep(2)
        return 'success'

class RetractGak(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: Retract Gak"
        rospy.loginfo('Enter to Retract Gak')
        raw_input()
        currPoseRight = self.davinciArmRight.getGripperPose()
        # Modify retraction point to be have offset about grasp point
        self.davinciArmRight.goToGripperPose(currPoseRight)
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

        rospy.loginfo('Enter to Release Grippers')
        raw_input()

        return 'success'

class IdentifyCutPoint(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'], output_keys = ['cutPoint'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.cutPointCurr = None

    def execute(self, userdata):
        print "State: IdentifyCutPoint"

        while True:
            rospy.sleep(2)
            # Break when cut point is published
        userdata.cutPoint = self.cutPointCurr
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
        return 'success'

class CuttingAction(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: CuttingAction"
        return 'success'

class CheckCut(MasterClass):
    def __init__(self, davinciArmLeft, davinciArmRight):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft

    def execute(self, userdata):
        print "State: CheckCut"
        pose = userdata.retractionStagingPose._obj
        pose.position.x += 0.002
        self.davinciArm.executeInterpolatedTrajectory(pose)
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
