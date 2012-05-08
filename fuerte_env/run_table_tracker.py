#!/usr/bin/env python
import subprocess
import fuerte_env
import roslib;
roslib.load_manifest("rospack")
import rospkg

rp = rospkg.RosPack()
pkg_path = rp.get_path("spinning_tabletop_detection")

subprocess.check_call(["%s/bin/test_tracker_ros"%pkg_path,"input_cloud:=/camera/rgb/points"], env = fuerte_env.make_fuerte_env())
