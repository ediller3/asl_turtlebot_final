#!/usr/bin/env python3
import rospy
from asl_turtlebot.msg import StringArray
import argparse
from time import sleep

animal_pub = rospy.Publisher('/rescue_animals', StringArray, queue_size=10)

def publish_animals_to_rescue(animals):
  msg = StringArray(data=animals)
  animal_pub.publish(msg)
  print(f"Sent animals: {msg}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--animals-list', nargs='+', default=[])
    args = parser.parse_args()
    rospy.init_node('rescue_request_publisher', anonymous=True)
    sleep(1)
    publish_animals_to_rescue(args.animals_list)
    sleep(1)
    