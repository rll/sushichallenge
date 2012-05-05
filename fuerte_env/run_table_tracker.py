import subprocess
import fuerte_env


subprocess.check_call(["/home/joschu/sushi_all/sushichallenge/spinning_tabletop_detection/bin/test_tracker_ros","input_cloud:=/camera/rgb/points"], env = fuerte_env.make_fuerte_env())