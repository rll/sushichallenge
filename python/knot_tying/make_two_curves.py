import image_proc.interactive_roi as roi
import os,cv2
from os.path import dirname, abspath,join
from image_proc import curves
import numpy as np

X = np.zeros((100,100),'uint8')+255
poly0 = roi.get_polyline(X,"curves")
poly1 = roi.get_polyline(X,"curves")
curve0 = curves.unif_resample(poly0,100)
curve1 = curves.unif_resample(poly1,100)

np.savetxt("curve0.txt",curve0)
np.savetxt("curve1.txt",curve1)