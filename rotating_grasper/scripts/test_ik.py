import roslib; roslib.load_manifest("rotating_grasper")
import rotating_grasper.srv as rgs
import rotating_grasp_server
import rospy

rospy.init_node("test_ik",disable_signals = True)
rospy.sleep(1)

req = rgs.RotatingGrasperRequest()
req.command.center.x = .4
req.command.center.y = 0
req.command.center.z = 1
req.command.rotation_rate = 1/30.
req.command.outward_angle = 0
req.command.header.stamp = rospy.Time.now()
req.command.header.frame_id = "base_footprint"
rotating_grasp_server.handle_rotating_grasp_ik(req)

