#!/usr/bin/env python3

import rospy
import smach, smach_ros
from std_msgs.msg import Int32

class AUTO_EXPLORE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['auto_exp_success', 'auto_exp_fail'])
        self.switchState = False
        self.switchTo = None
        rospy.Subscriber("/sup_state_change", Int32, self.callback)


    def callback(self, msg):
        self.switchTo = msg.data
        self.switchState = True

    def execute(self, userdata):
        self.switchState = False
        self.switchTo = None
        rate = rospy.Rate(10)
        rospy.loginfo('Executing state AUTO_EXPLORE')
        while not self.switchState:
            rate.sleep()
        
        if self.switchTo == 1:
            return 'auto_exp_fail'
        elif self.switchTo == 3:
            return 'auto_exp_success'
        else:
            rospy.loginfo("ERROR! Could not switch to %d."%(self.switchTo))

class DISCOVER(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_rescue'])
        self.switchState = False
        self.switchTo = None
        rospy.Subscriber("/sup_state_change", Int32, self.callback)


    def callback(self, msg):
        self.switchTo = msg.data
        self.switchState = True

    def execute(self, userdata):
        self.switchState = False
        self.switchTo = None
        rate = rospy.Rate(10)
        rospy.loginfo('Executing state DISCOVER')
        while not self.switchState:
            rate.sleep()

        if self.switchTo == 2:
            return 'start_rescue'
        else:
            rospy.loginfo("ERROR! Could not switch to %d."%(self.switchTo))

class RESCUE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rescue_complete'])
        self.switchState = False
        self.switchTo = None
        rospy.Subscriber("/sup_state_change", Int32, self.callback)


    def callback(self, msg):
        self.switchTo = msg.data
        self.switchState = True

    def execute(self, userdata):
        self.switchState = False
        self.switchTo = None
        rate = rospy.Rate(10)
        rospy.loginfo('Executing state RESCUE')
        while not self.switchState:
            rate.sleep()

        if self.switchTo == 3:
            return 'rescue_complete'
        else:
            rospy.loginfo("ERROR! Could not switch to %d."%(self.switchTo))

class RETURN(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['return_complete'])
        self.switchState = False
        self.switchTo = None
        rospy.Subscriber("/sup_state_change", Int32, self.callback)


    def callback(self, msg):
        self.switchTo = msg.data
        self.switchState = True

    def execute(self, userdata):
        self.switchState = False
        self.switchTo = None
        rate = rospy.Rate(10)
        rospy.loginfo('Executing state RETURN')
        while not self.switchState:
            rate.sleep()
        
        if self.switchTo == 1:
            return 'return_complete'
        else:
            rospy.loginfo("ERROR! Could not switch to %d."%(self.switchTo))

# main
def main():
    rospy.init_node('sup_smach')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('DISCOVER', DISCOVER(), 
                               transitions={'start_rescue':'RESCUE'})
        smach.StateMachine.add('RESCUE', RESCUE(), 
                               transitions={'rescue_complete':'RETURN'})
        smach.StateMachine.add('RETURN', RETURN(), 
                               transitions={'return_complete':'DISCOVER'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('sup_server', sm, '/SM_SUP')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()