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

class StateTestClass(smach.State):
    """ This is a the base test counterlass for the testing states.

        The excute method simply iterates through all the possible transitions
        from the state. """
    counter = 0

    def __init__(self):
        
        self.outcomes = None

        self.homePose = tfx.pose([0.08110923304266986, 0.019082072847487756, -0.07564648655601992],
            (-0.7296703092609553, 0.5879730580371108, -0.28914218075416975, 0.19561626239715652))
        self.retractionStagingPose = None
        self.graspPoint = None
        self.dropOffStagingPose = None
        self.dropOffPose = None

    def execute(self, userdata):        
        smach.loginfo(self.__class__.__name__ + " must be subclassed. It does nothing.")
        time.sleep(SLEEP_TIME)
        self.counter += 1
        if self.outcomes is None:
            self.outcomes = self.get_registered_outcomes()
        return self.outcomes[self.counter%len(self.outcomes)]




          # smach.StateMachine.add('START',
          #       Start(self.davinciArm),
          #       transitions={'success':'IDENTIFY_GRASP_POINT'})

          #   smach.StateMachine.add('IDENTIFY_GRASP_POINT',
          #       MoveToRetractionStagingArea(self.davinciArm),
          #       transitions={'success':'PLAN_TRAJ_TO_GRASP_POINT_RIGHT', 'failure': 'IDENTIFY_GRASP_POINT'}, remapping ={'graspPoint':'sm_data1'})

          #   smach.StateMachine.add('PLAN_TRAJ_TO_GRASP_POINT_RIGHT',
          #       HomePosition(self.davinciArm),
          #       transitions={'success':'MOVE_TO_GRASP_POINT', 'failure': 'PLAN_TRAJ_TO_GRASP_POINT_RIGHT'}, remapping ={'graspPointPlanning':'sm_data1'})

          #   smach.StateMachine.add('MOVE_TO_GRASP_POINT',
          #       MoveToGraspPoint(self.davinciArm),
          #       transitions={'success':'GRASP_GAK', 'failure':'HOME_POSITION_RIGHT'})

          #   smach.StateMachine.add('HOME_POSITION_RIGHT',
          #       MoveToGraspPoint(self.davinciArm),
          #       transitions={'success':'IDENTIFY_GRASP_POINT', 'failure':'HOME_POSITION_RIGHT'})

          #   smach.StateMachine.add('GRASP_GAK',
          #       GraspBlock(self.davinciArm),
          #       transitions={'success':'RETRACT_GAK'})

          #   smach.StateMachine.add('RETRACT_GAK',
          #       GraspBlock(self.davinciArm),
          #       transitions={'success':'CHECK_GRASP'})

          #   smach.StateMachine.add('CHECK_GRASP',
          #       CheckGrasp(self.davinciArm),
          #       transitions={'success':'IDENTIFY_CUT_POINT', 'failure': 'RELEASE_GRIPPERS_NO_GAK'})

          #   smach.StateMachine.add('RELEASE_GRIPPERS_NO_GAK',
          #       ReleaseGrippersNoBlock(self.davinciArm),
          #       transitions={'success':'HOME_POSITION_RIGHT', 'failure': 'RELEASE_GRIPPERS_NO_GAK'})

          #   smach.StateMachine.add('IDENTIFY_CUT_POINT',
          #       MoveToRetractionStagingArea(self.davinciArm),
          #       transitions={'success':'PLAN_TRAJ_TO_CUT_POINT_LEFT', 'failure': 'IDENTIFY_CUT_POINT'}, remapping ={'cutPoint':'sm_data2'})

          #   smach.StateMachine.add('PLAN_TRAJ_TO_CUT_POINT_LEFT',
          #       ReturnToRetractionStagingAreaWithBlock(self.davinciArm),
          #       transitions={'success':'MOVE_TO_CUT_POINT', 'failure': 'PLAN_TRAJ_TO_CUT_POINT_LEFT'}, remapping = {'cutPointPlanning':'sm_data2'})

          #   smach.StateMachine.add('MOVE_TO_CUT_POINT',
          #       MoveToDropOffStagingArea(self.davinciArm),
          #       transitions={'success':'CUTTING_ACTION', 'failure': 'HOME_POSITION_ELFT'}, remapping = {'dropOffStagingPose': 'sm_data3'})

          #   smach.StateMachine.add('CUTTING_ACTION',
          #       MoveToDropOffPoint(self.davinciArm),
          #       transitions={'success':'CHECK_CUT'})

          #   smach.StateMachine.add('CHECK_CUT',
          #       ReleaseGripper(self.davinciArm),
          #       transitions={'success':'SUCCESS', 'failure': 'HOME_POSITION_ELFT'})

          #   smach.StateMachine.add('HOME_POSITION_LEFT',
          #       MoveToGraspPoint(self.davinciArm),
          #       transitions={'success':'IDENTIFY_CUT_POINT', 'failure':'HOME_POSITION_LEFT'})


class Start(StateTestClass):
    def __init__(self, davinciArmRight, davinciArmLeft):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success'], output_keys = ['retractionStagingPose', 'graspPoint', 'dropOffStagingPose', 'dropOffPose'])
        self.davinciArmRight = davinciArmRight
        self.davinciArmLeft = davinciArmLeft
        self.counter = 0
        rospy.Subscriber("/triangle_pose_array", PoseArray, self.pose_array_callback)

        self.publisher = rospy.Publisher("/grasp_point", PoseStamped)
        self.triangle_polygon_publisher = rospy.Publisher("/triangle_polygon", PolygonStamped)


    def pose_array_callback(self, msg):
        poses = tfx.pose(msg)
        # sort the poses
        poses.sort(key = lambda p: p.position.x, reverse = True)
        self.poses = poses


    def publish_triangle_polygon(self, pose):
        pts = []
        pts.append(tfx.pose((0,0.0079,0)))
        pts.append(tfx.pose((0,-0.0079,0)))
        pts.append(tfx.pose((0,0, -0.013)))
        for i in range(len(pts)):
            # IPython.embed()
            pts[i] = (pose.as_tf()*pts[i]).position.msg.Point()
        polygon = Polygon()
        polygon_stamped = PolygonStamped()
        polygon.points = pts
        polygon_stamped.polygon = polygon
        polygon_stamped.header = pose.msg.Header()
        self.triangle_polygon_publisher.publish(polygon_stamped)


    def execute(self, userdata):
        print "State: Start"
        self.counter = StateTestClass.counter
        while True:
            rospy.sleep(0.1)
            if (self.poses != None ):
                if (len(self.poses) == 3):
                    break

        if self.counter%2 == 0:
            pose = self.poses[2]

            self.publish_triangle_polygon(pose)

            self.graspPoint = tfx.pose(pose, copy=True)

            self.graspPoint = raven_util.convertToFrame(self.graspPoint, '/two_remote_center_link')
            self.graspPoint.position.x += -0.005
            self.graspPoint.position.y += 0.012
            self.graspPoint.position.z += 0.013

            # rotation = tfx.rotation_tb(0,90,0) * tfx.rotation_tb(-30,0,0) * tfx.rotation_tb(0,0,5)
            # rotation = tfx.rotation_tb(0,90,0) * tfx.rotation_tb(-30,0,0)
            # self.graspPoint = self.graspPoint.as_tf()*rotation.as_pose()

            rotation = tfx.rotation_tb(0,180,0)*tfx.rotation_tb(-90,0,0)
            self.graspPoint.rotation = rotation


            self.publisher.publish(self.graspPoint.msg.PoseStamped())

            self.retractionStagingPose = tfx.pose(self.graspPoint, copy=True)
            self.retractionStagingPose.position.z += 0.03
            self.retractionStagingPose.position.x += 0.000
            self.dropOffStagingPose = tfx.pose(self.retractionStagingPose, copy = True)
            self.dropOffStagingPose.position.y += 0.01
            self.dropOffStagingPose.position.x += 0.052

            self.dropOffPose = tfx.pose(self.dropOffStagingPose, copy = True)
            self.dropOffPose.position.z += - 0.010
            self.counter += 1

        else:
            prev_retraction_staging_pose = self.retractionStagingPose

            pose = self.poses[1]

            self.publish_triangle_polygon(pose)

            self.graspPoint = tfx.pose(pose, copy=True)

            self.graspPoint = raven_util.convertToFrame(self.graspPoint, '/two_remote_center_link')
            self.graspPoint.position.x += -0.006
            self.graspPoint.position.y += 0.013
            self.graspPoint.position.z += 0.013

            # rotation = tfx.rotation_tb(0,90,0) * tfx.rotation_tb(-30,0,0) * tfx.rotation_tb(0,0,5)
            # rotation = tfx.rotation_tb(0,90,0) * tfx.rotation_tb(-30,0,0)
            # self.graspPoint = self.graspPoint.as_tf()*rotation.as_pose()
            rotation = tfx.rotation_tb(0,180,0)*tfx.rotation_tb(-90,0,0)
            self.graspPoint.rotation = rotation

            self.publisher.publish(self.graspPoint.msg.PoseStamped())

            self.retractionStagingPose = tfx.pose(self.graspPoint, copy=True)
            self.retractionStagingPose.position.z += 0.03
            self.retractionStagingPose.position.x += 0.000


            self.dropOffStagingPose = prev_retraction_staging_pose
            self.dropOffStagingPose.position.x += 0.0065

            self.dropOffPose = tfx.pose(self.dropOffStagingPose, copy = True)
            self.dropOffPose.position.z += - 0.010
            self.counter += 1


        # rospy.loginfo('Execute Start')
        # raw_input()

        userdata.retractionStagingPose = self.retractionStagingPose
        userdata.graspPoint = self.graspPoint
        userdata.dropOffStagingPose = self.dropOffStagingPose
        userdata.dropOffPose = self.dropOffPose
        # IPython.embed()
        # self.davinciArm.executeInterpolatedTrajectory(self.homePose)
        return 'success'

class MoveToRetractionStagingArea(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'], input_keys = ['retractionStagingPose'])
        self.davinciArm = davinciArm        

    def execute(self, userdata):
        print "State: MoveToRetractionStagingArea"

        # rospy.loginfo('Execute MoveToRetractionStagingArea')
        # raw_input()

        self.davinciArm.setGripperPositionDaVinci(0.3)
        pose = userdata.retractionStagingPose._obj
        self.davinciArm.executeInterpolatedTrajectory(pose)
        return 'success'

class HomePosition(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes = ['success', 'failure'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: HomePosition"
        self.davinciArm.executeInterpolatedTrajectory(self.homePose)

        rospy.loginfo('Wait After finishing home pose')
        raw_input()

        return 'success'

class MoveToGraspPoint(StateTestClass):
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

class GraspBlock(StateTestClass):
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

class CheckGrasp(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['graspPoint'])
        self.davinciArm = davinciArm
        # rospy.Subscriber("/triangle_pose_array", PoseArray, self.pose_array_callback)
        # self.failure_publisher = rospy.Publisher("/failed_grasp_text", Marker)
        self.poses = None
    
    def execute(self, userdata):
        print "State: CheckGrasp"
        # while(True):
        #     if self.poses != None:
        #         break

        # grasp_pose = userdata.graspPoint._obj
        # for pose in self.poses:
        #     pose = raven_util.convertToFrame(pose, '/two_remote_center_link')
        #     pose.position.x += -0.0065
        #     pose.position.y += 0.013
        #     pose.position.z += 0.013
        #     print 'DISTANCE ', pose.position.distance(grasp_pose.position)
        #     if pose.position.distance(grasp_pose.position) < 0.006 or len(self.poses)!=3:
        #         # IPython.embed()
        #         m = Marker()
        #         m.id = 1
        #         m.type  = Marker.TEXT_VIEW_FACING
        #         m.text = "GRASP FAILED"
        #         m.pose =  grasp_pose.msg.Pose()
        #         m.header = grasp_pose.msg.Header()
        #         m.scale.x = 0.01
        #         m.scale.y = 0.01
        #         m.scale.z = 0.01
        #         m.color.a = 1
        #         m.color.r = 0
        #         m.color.g = 0
        #         m.color.b = 1
        #         m.lifetime = rospy.rostime.Duration(5)
        #         self.failure_publisher.publish(m)
                # return 'failure'

        return 'success'

    def pose_array_callback(self, msg):
        poses = tfx.pose(msg)
        # sort the poses
        poses.sort(key = lambda p: p.position.x, reverse = True)
        self.poses = poses


class ReleaseGrippersNoBlock(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.davinciArm = davinciArm
    
    def execute(self, userdata):
        print "State: ReleaseGrippersNoBlock"
        return 'success'

class ReturnToRetractionStagingAreaWithBlock(StateTestClass):
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

class MoveToDropOffStagingArea(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys = ['dropOffStagingPose'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: MoveToDropOffStagingArea"

        pose = userdata.dropOffStagingPose._obj
        self.davinciArm.executeInterpolatedTrajectory(pose)
        return 'success'

class MoveToDropOffPoint(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys = ['dropOffPose'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: MoveToDropOffPoint"
        pose = userdata.dropOffPose._obj
        self.davinciArm.executeInterpolatedTrajectory(pose)
        return 'success'

class ReleaseGripper(StateTestClass):
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

class CheckDropOff(StateTestClass):
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

class Abort(StateTestClass):
    def __init__(self, davinciArm):
        super(self.__class__, self).__init__()
        smach.State.__init__(self, outcomes=['failure'])
        self.davinciArm = davinciArm

    def execute(self, userdata):
        print "State: Abort"
        return 'success'