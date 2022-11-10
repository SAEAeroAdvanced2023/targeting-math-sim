import numpy as np
import pandas as pd
import copy
import os, shutil
from Transformation_matrices import Transformation_translation, Transformation_rotation,camera_calibration_matrix, camera_calibration_matrix_2
#from pythonProject_1.Functions import Transformation_translation



def coor_camera_to_inertial_frame (x,y,z,roll,yaw,pitch,g_roll, g_yaw, g_pitch,calibration):

    C = np.array([[0], [0], [0], [1]])
#camera calibration
    if(calibration):
        CCM = camera_calibration_matrix_2()

        CCM_inv = np.linalg.inv(CCM)
        column_to_be_added = np.array([[0], [0], [0]])

        # Adding column to array using append() method
        CCM_inv = np.append(CCM_inv, column_to_be_added, axis=1)
        newrow = np.array([0, 0, 0, 1])
        CCM_inv= np.vstack([CCM_inv, newrow])

#given
    #target coord in camera
    pix = np.array([[0],[400],[1],[1]]) #x_pix,y_pix, l= depth, last is always 1
    #distance of center of rotation of gimbal from centroid of PA
    g_dist = np.array([0,0,0])
    #distance of camera center vision from center of rotation of gimbal
    c_dis = np.array([0,0,0])

#########
    trans_c = Transformation_translation(g_dist[0], g_dist[1], g_dist[2])
    rot_g = Transformation_rotation(g_roll, g_yaw, g_pitch)
    trans_g = Transformation_translation(g_dist[0], g_dist[1], g_dist[2])
    rot_v = Transformation_rotation(roll,yaw,pitch)
    trans_i = Transformation_translation(x, y, z)
    Z_cc = np.linalg.multi_dot([trans_c[1],rot_g[0],trans_g[1],rot_v[0],trans_i[0],C])[2]
    Z_obj = np.linalg.multi_dot([CCM_inv,trans_c[1],rot_g[0],trans_g[1],rot_v[0],trans_i[0],pix])[2]
    l = Z_cc/(Z_cc-Z_obj)
########

    #depth (given by lidar and other method)
    d =np.array([[l[0],0,0,0],[0,l[0],0,0],[0,0,l[0],0],[0,0,0,1]])
    target_pix = np.dot(d,pix)

    #pixel_to_coord
    #CCM_inv=camera_calibration_matrix(1,1,0,2,3)
    camera_frame = np.dot(CCM_inv, target_pix)

    #transformation_camera_gimbal
    trans_c = Transformation_translation(g_dist[0], g_dist[1], g_dist[2])
    gimbal_frame = np.dot(trans_c[1], camera_frame)


    # Transformation_gimbal_body
    rot_g = Transformation_rotation(g_roll, g_yaw, g_pitch)
    gimbal_frame = np.dot(rot_g[0], gimbal_frame)

    trans_g = Transformation_translation(g_dist[0], g_dist[1], g_dist[2])
    body_frame= np.dot(trans_g[1], gimbal_frame)

    #Transformation_body_vehicule
    rot_v = Transformation_rotation(roll,yaw,pitch)

    vehicule_frame = np.dot(rot_v[0],body_frame)




    #Transformation_vehicule_inertial
    trans_i = Transformation_translation(x, y, z)
    inertial_frame= np.dot(trans_i[0], vehicule_frame)

    #either use trans_i[0] with GPS coordinate of PA or use trans_i[1] with distance of inertial point from vehicule negative GPS coordinate (-x,-y,-z)



#results
    target_location  = inertial_frame[:3]
    latitude = inertial_frame[0]
    longitude = inertial_frame[1]
    height = inertial_frame[2]

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