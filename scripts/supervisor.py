#!/usr/bin/env python3

from enum import Enum

import rospy
import numpy as np
from asl_turtlebot.msg import DetectedObject, DetectedObjectList, StringArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Float32MultiArray, String, Int32
from visualization_msgs.msg import Marker
import tf

class Mode(Enum):
    """State machine modes. Feel free to change."""
    AUTO_EXPLORE = 0
    DISCOVER = 1
    RESCUE = 2
    RETURN = 3

class Supervisor:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('turtlebot_supervisor', anonymous=True)

        self.near_thresh = 0.25
        self.mode = Mode.DISCOVER
        self.prev_mode = None  # For printing purposes
        self.origin = (3.15, 1.6, 0)
        self.x = self.origin[0]
        self.y = self.origin[1]
        self.theta = self.origin[2]

        # Keep track of discovered & rescued animals
        self.animals_discovered = {}
        self.animals_to_rescue = []
        self.animal_types = [
            "dog",
            "cat",
            "giraffe",
            "horse",
            "elephant"
        ]

        self.start_time = rospy.get_rostime()

        self.waypoints =    [  (3.4233358681643704, 2.0164839470015576, 0.0), #cat
                               (2.5556310905569024, 2.8031463340039773, 3.0984228736087127),
                               (0.7729086841574924, 2.8022618945243964, -2.2793409282702535),
                               (0.3196694884880853, 1.755398573112974, 0.12193387175107573),
                               (0.5488329194855261, 1.6921026397813366, 1.4297736114376145), #dog
                               (0.2879257437502593, 1.7256991482180148, -1.5415648632689034),
                               (2.307002531909424, 1.9074444990466874, 3.1008738823655455),
                               (2.4836124329990086, 0.3763751459363931, -1.2679121895410606), 
                               (0.6120595033067794, 0.3910181634118649, -2.8731482725221955) 
                            ]

        self.pos_g = None 

        ########## PUBLISHERS ##########

        # Marker publisher
        self.marker_publisher = rospy.Publisher('/marker_topic', Marker, queue_size=10)
        self.marker_count = 0

        # Goal nav publisher
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)

        #animal announcer publisher
        self.animal_announcer_publisher = rospy.Publisher('/animal_announcer', String, queue_size=10) 

        self.state_change_pub = rospy.Publisher("/sup_state_change", Int32, queue_size=10)

        ########## SUBSCRIBERS ##########

        # Process new found animals
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.object_detected_callback)
        rospy.Subscriber('/print_animals', String, self.print_discovered_objects_callback)
        rospy.Subscriber('/rescue_animals', StringArray, self.initiate_animal_rescue_callback)

        # Position callback
        rospy.Subscriber('/cur_pos', Pose2D, self.current_position_callback)
        rospy.Subscriber("/state_change", Int32, self.state_change_callback)

    def switchMode(self, new_mode):
        rospy.loginfo("Switching from %s -> %s", self.mode, new_mode)
        self.mode = new_mode
        msg = Int32()
        msg.data = self.mode.value
        self.state_change_pub.publish(msg)

    ########## SUBSCRIBER CALLBACKS ##########

    def state_change_callback(self, msg):
        if self.mode == Mode.DISCOVER:
                return
        if msg.data != 0:
            return
        if np.linalg.norm(np.array([self.x - self.pos_g[0], self.y - self.pos_g[1]])) < self.near_thresh:
            if self.mode == Mode.RETURN:
                self.switchMode(Mode.DISCOVER)
                return

            if self.mode == Mode.AUTO_EXPLORE:
                if len(self.waypoints) == 0:
                    if len(self.animals_discovered) != 5:
                        rospy.loginfo("Warning! Not all animals discovered. Manual discovery reccomended.")
                        self.switchMode(Mode.DISCOVER)
                        return
                    else:
                        rospy.loginfo("AUTO_EXPLORE succesful, all animals discovered.")
                        self.switchMode(Mode.RETURN) #do we need to return after discovering all animals?
                        return
                else:
                    self.navigate_to_next_waypoint()
                    return
            
            rospy.loginfo("Rescuing animal. Sleeping...")
            rospy.sleep(5)
            rospy.loginfo("Rescued animal!")
            if len(self.animals_to_rescue) == 0:
                self.pos_g = self.origin
                self.start_time = rospy.get_rostime()
                self.switchMode(Mode.RETURN)
                return
            else:
                self.navigate_to_next_animal()
        else:
            self.start_time = rospy.get_rostime()
            

    def object_detected_callback(self, msg):
        valid_msgs = [m for m in msg.ob_msgs if m.name in self.animal_types]
        for m in valid_msgs:
            if m.name in self.animals_discovered and m.confidence <= self.animals_discovered[m.name][0]:
                continue
            
            self.animals_discovered[m.name] = [m.confidence, (self.x, self.y, self.theta)]
            rospy.loginfo(f"Discovered animal: {m.name} @ {self.animals_discovered[m.name]} (cur pos: {self.x}, {self.y}, {self.theta})")

            marker_id = list(self.animals_discovered).index(m.name)
            self.delete_marker(marker_id)
            self.send_marker(m.name, marker_id, *self.animals_discovered[m.name][1])
            if m.name == 'cat':
                self.animal_announcer_publisher.publish("meow!")
            elif m.name=='dog':
                self.animal_announcer_publisher.publish("woof!")
            elif m.name == 'horse':
                self.animal_announcer_publisher.publish("nehh!")
            elif m.name == 'bird':
                self.animal_announcer_publisher.publish("tweet!")
            elif m.name == "elephant":
                self.animal_announcer_publisher.publish("thwamp!")

    def delete_marker(self, id):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.id = id
        marker.action = 2
        self.marker_publisher.publish(marker)
            
    def send_marker(self, name, marker_id, x, y, z):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.id = marker_id
        marker.type = 9
        marker.text = name
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.2
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 0.9
        self.marker_publisher.publish(marker)

    def print_discovered_objects_callback(self, msg):
        for name, coords in self.animals_discovered.items():
            rospy.loginfo(f"Discovered animal: {name} @ {coords}")

    def initiate_animal_rescue_callback(self, msg):
        rospy.loginfo(f"received animals to rescue: {msg.data}")
        for animal in msg.data:
            if animal not in self.animals_discovered:
                rospy.loginfo(f"Have not discovered {animal} yet. Aborting rescue.")
                return
        self.animals_to_rescue = msg.data
        rospy.loginfo(f"Saving animals: {msg.data}")
        self.switchMode(Mode.RESCUE)
        self.navigate_to_next_animal()

    def navigate_to_next_animal(self):
        dists = [np.linalg.norm([self.x - self.animals_discovered[name][1][0], self.y - self.animals_discovered[name][1][1]]) for name in self.animals_to_rescue]
        min_idx = dists.index(min(dists))
        self.pos_g = self.animals_discovered[self.animals_to_rescue[min_idx]][1]
        self.animals_to_rescue.pop(min_idx)
        self.start_time = rospy.get_rostime()

    def navigate_to_next_waypoint(self):
        rospy.loginfo("Navigating to next waypoint: ")
        rospy.loginfo(self.waypoints[0])
        self.pos_g = self.waypoints[0]
        self.waypoints.pop(0)
        self.start_time = rospy.get_rostime()

    def current_position_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta


    ########## STATE MACHINE ACTIONS ##########

    ########## Code starts here ##########
    # Feel free to change the code here. You may or may not find these functions
    # useful. There is no single "correct implementation".

    def publish_goal_pose(self):
        """ sends the current desired pose to the navigator """
        if self.pos_g is not None:
            pose_g_msg = Pose2D()
            pose_g_msg.x = self.pos_g[0]
            pose_g_msg.y = self.pos_g[1]
            pose_g_msg.theta = self.pos_g[2]
            self.nav_goal_publisher.publish(pose_g_msg)
            rospy.loginfo(f"Sent goal {self.pos_g}")


    ########## Code ends here ##########


    ########## STATE MACHINE LOOP ##########

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        # logs the current mode
        if self.prev_mode != self.mode:
            rospy.loginfo("Current mode: %s", self.mode)
            self.prev_mode = self.mode

        ########## Code starts here ##########
        # TODO: Currently the state machine will just go to the pose without stopping
        #       at the stop sign.

        if self.mode == Mode.DISCOVER:
            pass
        elif self.mode == Mode.AUTO_EXPLORE or self.mode == Mode.RESCUE or self.mode == Mode.RETURN:
            t = rospy.get_rostime()
            if (t - self.start_time).to_sec() < 2.0:
                self.publish_goal_pose()
        else:
            raise Exception("This mode is not supported: {}".format(str(self.mode)))

        ############ Code ends here ############

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
