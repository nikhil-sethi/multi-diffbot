#!/usr/bin/env python3

# Just some random tests. This file can be deleted.


import numpy as np
# import rospy
import cv2

color_img = cv2.imread('bullproof_ws/src/bullproof-ros/scripts/smiley.jpeg')
grey_img = np.mean(color_img, axis=2)
bitmap = np.round(-grey_img/255+1).astype(int)
occupancy_map = bitmap*100


# max pooling
# M, N = bitmap.shape
# K = 5
# L = 5
# MK = M // K
# NL = N // L
# max_pool = bitmap[:MK*K, :NL*L].reshape(MK, K, NL, L).max(axis=(1, 3))

# max_pool_show = cv2.resize(max_pool, (960, 540))
cv2.imshow('image', occupancy_map.astype(float))
cv2.waitKey(0)  

















