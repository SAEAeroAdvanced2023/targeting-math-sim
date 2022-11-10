import numpy as np
import pandas as pd
import copy
import os, shutil
from Transformation_matrices import Transformation_translation, camera_calibration_matrix_2, Transformation_rotation_2, isect_line_plane_v3
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
    pix = np.array([[963],[616],[1],[1]]) #x_pix,y_pix, l= depth, last is always 1
    #distance of center of rotation of gimbal from centroid of PA
    g_dist = np.array([0,0,0])
    #distance of camera center vision from center of rotation of gimbal
    c_dis = np.array([0,0,0])
    f = 1450*0.0003741
    focal_lenght_meters =np.array([[f,0,0,0],[0,f,0,0],[0,0,f,0],[0,0,0,1]])

#########
    trans_c = Transformation_translation(g_dist[0], g_dist[1], g_dist[2])
    rot_g = Transformation_rotation_2(g_roll, g_pitch, g_yaw)
    trans_g = Transformation_translation(g_dist[0], g_dist[1], g_dist[2])
    rot_v = Transformation_rotation_2(roll,pitch,yaw)
    trans_i = Transformation_translation(x, y, z)
    P_cc = np.dot(np.linalg.inv(np.linalg.multi_dot([trans_c[1],rot_g,trans_g[1],rot_v,trans_i[0]])),C)
    q_obj = np.linalg.multi_dot([np.linalg.inv(np.linalg.multi_dot([CCM,trans_c[1],rot_g,trans_g[1],rot_v,trans_i[0]])),focal_lenght_meters,pix])
    t = isect_line_plane_v3(P_cc[:3],q_obj[:3],[1,1,0],[0,0,1])
    #l = Z_cc/(Z_cc-Z_obj)
    t_norm = np.linalg.norm((t-P_cc[:3]),2)
    r = np.sqrt(np.square(pix[0][0]-CCM[0][2])+np.square(pix[1][0]-CCM[1][2]))
    Beta = np.arctan((r/CCM[0][0])) #angle between depth and line to target from center of optical lens
    l = t_norm* np.cos(Beta)
    T = np.linalg.inv(np.linalg.multi_dot([CCM,trans_c[1],rot_g,trans_g[1],rot_v,trans_i[0]]))
########
    Q =np.array([[l,0,0,0],[0,l,0,0],[0,0,l,0],[0,0,0,1]])

    inertial_frame_coord =np.linalg.multi_dot([T,Q,pix])
#results
    target_location  = inertial_frame_coord[:3]
    latitude = inertial_frame_coord[0]
    longitude = inertial_frame_coord[1]
    height = inertial_frame_coord[2]

    return latitude,longitude,height
if __name__ == '__main__':
    coor_camera_to_inertial_frame (x = 0,y = 0, z = -40,roll=0,yaw=0,pitch=-np.pi,g_roll=0, g_yaw=0, g_pitch=0, calibration = True)
#xyz = longitude,latitude and height of PA
#g_a_dist = distance in x y z from the centroi of PA to gimbal
#g_roll, g_yaw, g_pitch = gimbal's rotation

#roll,yaw and pitch follow right hand rule convention:
#pitch up positive
#yaw counterclockwise positive
#roll to the right positive

#angles in rad