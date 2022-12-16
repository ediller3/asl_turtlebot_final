#!/usr/bin/env python3

import rospy
import smach, smach_ros
from std_msgs.msg import Int32

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['idle_to_idle', 'idle_to_align','idle_to_track','idle_to_park'])
        self.switchState = False
        self.switchTo = None
        rospy.Subscriber("/state_change", Int32, self.callback)


    def callback(self, msg):
        self.switchTo = msg.data
        self.switchState = True

    def execute(self, userdata):
        self.switchState = False
        self.switchTo = None
        rate = rospy.Rate(10)
        rospy.loginfo('Executing state Idle')
        while not self.switchState:
            rate.sleep()
        
        if self.switchTo == 0:
            return 'idle_to_idle'
        if self.switchTo == 1:
            return 'idle_to_align'
        elif self.switchTo == 2:
            return 'idle_to_track'
        elif self.switchTo == 3:
            return 'idle_to_park'
        else:
            rospy.loginfo("ERROR! Could not switch to %d."%(self.switchTo))

class Align(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['align_to_idle', 'align_to_align', 'align_to_track','align_to_park'])
        self.switchState = False
        self.switchTo = None
        rospy.Subscriber("/state_change", Int32, self.callback)


    def callback(self, msg):
        self.switchTo = msg.data
        self.switchState = True

    def execute(self, userdata):
        self.switchState = False
        self.switchTo = None
        rate = rospy.Rate(10)
        rospy.loginfo('Executing state Align')
        while not self.switchState:
            rate.sleep()

        if self.switchTo == 0:
            return 'align_to_idle'
        if self.switchTo == 1:
            return 'align_to_align'
        elif self.switchTo == 2:
            return 'align_to_track'
        elif self.switchTo == 3:
            return 'align_to_park'
        else:
            rospy.loginfo("ERROR! Could not switch to %d."%(self.switchTo))

class Track(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['track_to_idle','track_to_park', 'track_to_align', 'track_to_track'])
        self.switchState = False
        self.switchTo = None
        rospy.Subscriber("/state_change", Int32, self.callback)


    def callback(self, msg):
        self.switchTo = msg.data
        self.switchState = True

    def execute(self, userdata):
        self.switchState = False
        self.switchTo = None
        rate = rospy.Rate(10)
        rospy.loginfo('Executing state Track')
        while not self.switchState:
            rate.sleep()

        if self.switchTo == 0:
            return 'track_to_idle'
        if self.switchTo == 1:
            return 'track_to_align'
        if self.switchTo == 2:
            return 'track_to_track' 
        elif self.switchTo == 3:
            return 'track_to_park'
        else:
            rospy.loginfo("ERROR! Could not switch to %d."%(self.switchTo))

class Park(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['park_to_idle', 'park_to_park'])
        self.switchState = False
        self.switchTo = None
        rospy.Subscriber("/state_change", Int32, self.callback)


    def callback(self, msg):
        self.switchTo = msg.data
        self.switchState = True

    def execute(self, userdata):
        self.switchState = False
        self.switchTo = None
        rate = rospy.Rate(10)
        rospy.loginfo('Executing state Park')
        while not self.switchState:
            rate.sleep()
        
        if self.switchTo == 0:
            return 'park_to_idle'
        elif self.switchTo == 3:
            return 'park_to_park'
        else:
            rospy.loginfo("ERROR! Could not switch to %d."%(self.switchTo))

# main
def main():
    rospy.init_node('nav_smach')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'idle_to_idle': 'IDLE',
                                            'idle_to_align':'ALIGN', 
                                            'idle_to_track':'TRACK',
                                            'idle_to_park':'PARK'})
        smach.StateMachine.add('ALIGN', Align(), 
                               transitions={'align_to_idle':'IDLE', 
                                            'align_to_align':'ALIGN',
                                            'align_to_track':'TRACK',
                                            'align_to_park':'PARK'})
        smach.StateMachine.add('TRACK', Track(), 
                               transitions={'track_to_idle':'IDLE',
                                            'track_to_align':'ALIGN',
                                            'track_to_track':'TRACK',
                                            'track_to_park':'PARK'})
        smach.StateMachine.add('PARK', Park(), 
                               transitions={'park_to_idle':'IDLE',
                                            'park_to_park':'PARK'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('nav_server', sm, '/SM_NAV')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()