#!/usr/bin/env python
"""
Author: Lorenzo Riano <lorenzo.riano@gmail.com>
"""
PKG = "sushi_sm"
import roslib; roslib.load_manifest(PKG)
import rospy
from pr2_python.head import Head
import yaml
import dynamic_reconfigure.client
import sys

def look_at_point(head, shelves, which_shelf, 
                  filter_x, filter_y, filter_z):
    max_height = shelves["heights"][which_shelf]['max']
    min_height = shelves["heights"][which_shelf]['min']
    width = shelves['width']
    depth = shelves['depth']
    
    x = 0.5
    y = 0.0
    z = min_height
    head.look_at_relative_point(x,y,z)
    
    params = { 'filter_limit_min' : 0, 'filter_limit_max' : 1.0 + depth}
    filter_x.update_configuration(params)
    
    params = { 'filter_limit_min' : -width/2., 'filter_limit_max' : width/2. }
    filter_y.update_configuration(params)
    
    params = { 'filter_limit_min' : min_height-0.2, 'filter_limit_max' : max_height+0.2 }    
    filter_z.update_configuration(params)


rospy.init_node("test_furniture_heights", anonymous=True)
DIR = roslib.packages.get_pkg_dir(PKG, required=True) + "/config/"
stream = file(DIR+"furniture_dims.yaml")
poses = yaml.load(stream)
filter_x = dynamic_reconfigure.client.Client("/pcl_filters/psx")
filter_y = dynamic_reconfigure.client.Client("/pcl_filters/psy")
filter_z = dynamic_reconfigure.client.Client("/pcl_filters/psz")
head = Head()

if len(sys.argv) > 1:
    arg_num = int(sys.argv[1])
else:
    arg_num = 0
rospy.loginfo("Using shelf %d", arg_num)
look_at_point(head, poses['shelves'], arg_num, filter_x, filter_y, filter_z)
    