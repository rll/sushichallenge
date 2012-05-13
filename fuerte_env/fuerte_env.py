
import os,sys,subprocess

def make_fuerte_env():
    versionstr = sys.version[:3]
    return dict(
        ROS_MASTER_URI = os.environ["ROS_MASTER_URI"],
        PATH = "/opt/ros/fuerte/bin:%s"%os.environ["PATH"],
        ROS_VERSION = "fuerte",
        PYTHONPATH = "/opt/ros/fuerte/lib/python%s/dist-packages"%versionstr,
        ROS_PACKAGE_PATH = "/opt/ros/fuerte/share:/opt/ros/fuerte/stacks")
