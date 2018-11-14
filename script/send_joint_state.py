#!/usr/bin/env python

"""Sends a single JointState message on a user specified topic."""

import argparse
import sys

import rospy
from sensor_msgs.msg import JointState


def send_state(topic, joint_angles):
    """Publish JointState message.

    :param topic topic on which to publish
    :param joint_angles joint angles to send
    """
    pub = rospy.Publisher(topic, JointState, queue_size=10)
    rospy.init_node("joint_state_test", anonymous=True)

    js = JointState()
    js.position = joint_angles
    pub.publish(js)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("topic", help="Topic to publish joint state on")
    parser.add_argument("joints", nargs=7, type=float, help="Joint angles")
    args = parser.parse_args()

    send_state(args.topic, args.joints)

    return 0


if __name__ == "__main__":
    sys.exit(main())
