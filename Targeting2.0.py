import numpy as np
import pandas as pd
import copy
import os, shutil
from Transformation_matrices import Transformation_translation, camera_calibration_matrix_2, Transformation_rotation_2
#from pythonProject_1.Functions import Transformation_translation



def coor_camera_to_inertial_frame (x,y,z,roll,yaw,pitch,g_roll, g_yaw, g_pitch,calibration):

    C = np.array([[0], [0], [0], [1]])
#camera calibration
    if(calibration):
        CCM = camera_calibration_matrix_2()
        column_to_be_added = np.array([[0], [0], [0]])

        # Adding column to array using append() method
        CCM = np.append(CCM, column_to_be_added, axis=1)
        newrow = np.array([0, 0, 0, 1])
        CCM = np.vstack([CCM, newrow])

#given
    #target coord in camera
    pix = np.array([[0],[400],[1],[1]]) #x_pix,y_pix, l= depth, last is always 1
    #distance of center of rotation of gimbal from centroid of PA
    g_dist = np.array([0,0,0])
    #distance of camera center vision from center of rotation of gimbal
    c_dis = np.array([0,0,0])

#########
    trans_c = Transformation_translation(g_dist[0], g_dist[1], g_dist[2])
    rot_g = Transformation_rotation_2(g_roll, g_yaw, g_pitch)
    trans_g = Transformation_translation(g_dist[0], g_dist[1], g_dist[2])
    rot_v = Transformation_rotation_2(roll,yaw,pitch)
    trans_i = Transformation_translation(x, y, z)
    Z_cc = np.dot(np.linalg.inv(np.linalg.multi_dot([trans_c[1],rot_g,trans_g[1],rot_v,trans_i[0]])),C)[2]
    Z_obj = np.dot(np.linalg.inv(np.linalg.multi_dot([CCM,trans_c[1],rot_g,trans_g[1],rot_v,trans_i[0]])),pix)[2]
    l = Z_cc/(Z_cc-Z_obj)
    T = np.linalg.inv(np.linalg.multi_dot([CCM,trans_c[1],rot_g,trans_g[1],rot_v,trans_i[0]]))
########
    Q =np.array([[l[0],0,0,0],[0,l[0],0,0],[0,0,l[0],0],[0,0,0,1]])

    inertial_frame_coord =np.linalg.multi_dot([T,Q,pix])
#results
    target_location  = inertial_frame_coord[:3]
    latitude = inertial_frame_coord[0]
    longitude = inertial_frame_coord[1]
    height = inertial_frame_coord[2]

    return latitude,longitude,height
if __name__ == '__main__':
    coor_camera_to_inertial_frame (x = 0,y = 0, z = 0,roll=0,yaw=0,pitch=0,g_roll=0, g_yaw=0, g_pitch=0, calibration = True)
#xyz = longitude,latitude and height of PA
#g_a_dist = distance in x y z from the centroi of PA to gimbal
#g_roll, g_yaw, g_pitch = gimbal's rotation

#roll,yaw and pitch follow right hand rule convention:
#pitch up positive
#yaw counterclockwise positive
#roll to the right positive

#angles in rad