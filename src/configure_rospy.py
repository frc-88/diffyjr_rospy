#!/usr/bin/env python3

import os
import yaml
import logging
import importlib
import rospy
import wpilib

def configure_rospy():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    ros_environment_config_path = os.path.join(dir_path, "ros_environment.yaml")
    with open(ros_environment_config_path) as file:
        config = yaml.safe_load(file)

    os.environ["ROS_IP"] = config["ros_ip"]
    os.environ["ROS_MASTER_URI"] = config["ros_master_uri"]
    os.environ["ROS_PYTHON_LOG_CONFIG_FILE"] = os.path.join(dir_path, "python_logging.yaml")

    importlib.reload(logging)

    print("rospy is configured")
