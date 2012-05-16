import numpy as np
import reachability

pose_array = np.loadtxt("pose_array.txt")
reachability.get_base_positions(pose_array)