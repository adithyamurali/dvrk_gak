#!/usr/bin/env python

# Authors: Adithya Murali and Siddarth Sen
# UC Berkeley, 2014

import roslib
import time
import smach
import rospy

from base_classes import *

from davinci_utils import raven_util
from davinci_utils import raven_constants
from davinci_trajectory.raven_controller import RavenController
from davinci_trajectory.raven_arm import RavenArm

class MasterClass:
    def __init__(self):
        rospy.init_node('Master_SM_Gak',anonymous=False)
        self.state_machine = smach.StateMachine(outcomes=['SUCCESS', 'FAILURE'])
        self.davinciArmRight = RavenArm(raven_constants.Arm.Right)
        self.davinciArmLeft = RavenArm(raven_constants.Arm.Left)
        self.setup_state_machine()

    def setup_state_machine(self):
        smach.loginfo("Initializing state machine")
        rospy.sleep(1)

        with self.state_machine:
            smach.StateMachine.add('START',
                Start(self.davinciArm),
                transitions={'success':'IDENTIFY_GRASP_POINT'})

            smach.StateMachine.add('IDENTIFY_GRASP_POINT',
                MoveToRetractionStagingArea(self.davinciArm),
                transitions={'success':'PLAN_TRAJ_TO_GRASP_POINT_RIGHT', 'failure': 'IDENTIFY_GRASP_POINT'}, remapping ={'graspPoint':'sm_data1'})

            smach.StateMachine.add('PLAN_TRAJ_TO_GRASP_POINT_RIGHT',
                HomePosition(self.davinciArm),
                transitions={'success':'MOVE_TO_GRASP_POINT', 'failure': 'PLAN_TRAJ_TO_GRASP_POINT_RIGHT'}, remapping ={'graspPointPlanning':'sm_data1'})

            smach.StateMachine.add('MOVE_TO_GRASP_POINT',
                MoveToGraspPoint(self.davinciArm),
                transitions={'success':'GRASP_GAK', 'failure':'HOME_POSITION_RIGHT'})

            smach.StateMachine.add('HOME_POSITION_RIGHT',
                MoveToGraspPoint(self.davinciArm),
                transitions={'success':'IDENTIFY_GRASP_POINT', 'failure':'HOME_POSITION_RIGHT'})

            smach.StateMachine.add('GRASP_GAK',
                GraspBlock(self.davinciArm),
                transitions={'success':'RETRACT_GAK'})

            smach.StateMachine.add('RETRACT_GAK',
                GraspBlock(self.davinciArm),
                transitions={'success':'CHECK_GRASP'})

            smach.StateMachine.add('CHECK_GRASP',
                CheckGrasp(self.davinciArm),
                transitions={'success':'IDENTIFY_CUT_POINT', 'failure': 'RELEASE_GRIPPERS_NO_GAK'})

            smach.StateMachine.add('RELEASE_GRIPPERS_NO_GAK',
                ReleaseGrippersNoBlock(self.davinciArm),
                transitions={'success':'HOME_POSITION_RIGHT', 'failure': 'RELEASE_GRIPPERS_NO_GAK'})

            smach.StateMachine.add('IDENTIFY_CUT_POINT',
                MoveToRetractionStagingArea(self.davinciArm),
                transitions={'success':'PLAN_TRAJ_TO_CUT_POINT_LEFT', 'failure': 'IDENTIFY_CUT_POINT'}, remapping ={'cutPoint':'sm_data2'})

            smach.StateMachine.add('PLAN_TRAJ_TO_CUT_POINT_LEFT',
                ReturnToRetractionStagingAreaWithBlock(self.davinciArm),
                transitions={'success':'MOVE_TO_CUT_POINT', 'failure': 'PLAN_TRAJ_TO_CUT_POINT_LEFT'}, remapping = {'cutPointPlanning':'sm_data2'})

            smach.StateMachine.add('MOVE_TO_CUT_POINT',
                MoveToDropOffStagingArea(self.davinciArm),
                transitions={'success':'CUTTING_ACTION', 'failure': 'HOME_POSITION_ELFT'}, remapping = {'dropOffStagingPose': 'sm_data3'})

            smach.StateMachine.add('CUTTING_ACTION',
                MoveToDropOffPoint(self.davinciArm),
                transitions={'success':'CHECK_CUT'})

            smach.StateMachine.add('CHECK_CUT',
                ReleaseGripper(self.davinciArm),
                transitions={'success':'SUCCESS', 'failure': 'HOME_POSITION_ELFT'})

            smach.StateMachine.add('HOME_POSITION_LEFT',
                MoveToGraspPoint(self.davinciArm),
                transitions={'success':'IDENTIFY_CUT_POINT', 'failure':'HOME_POSITION_LEFT'})

            # smach.StateMachine.add('ABORT', Abort(self.davinciArm), transitions={'failure': 'FAILURE'})

    def run(self):
        self.davinciArmRight.start()
        self.davinciArmLeft.start()
        rospy.sleep(2)

        rate = rospy.Rate(1)
        try:
            self.state_machine.execute()
        except Exception, e:
            print e

        while not rospy.is_shutdown():
            if not self.state_machine.is_running():
                print "State Machine stopped"
                rospy.sleep(0.5)
                rospy.signal_shutdown('state machines finished')
                break
            rate.sleep()

        self.davinciArm.stop()

def main():
    smach.loginfo("Starting main...")
    master = MasterClass()
    master.run()


if __name__ == '__main__':
    main()



