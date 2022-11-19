import math

import numpy as np
import pandas as pd
import copy
import os, shutil
from Transformation_matrices import Transformation_translation, camera_calibration_matrix_2, Transformation_rotation, isect_line_plane_v3,gimbal_rot
import matplotlib.pyplot as plt



def coor_camera_to_inertial_frame (x,y,z,roll,yaw,pitch,g_roll, g_yaw, g_pitch,calibration):
    pitch = np.pi+pitch #insert a pitch matrix to replace this
    roll = -roll
    yaw = yaw

    #g_pitch = np.pi + g_pitch
    g_roll = -g_roll
    g_yaw = g_yaw



    C = np.array([[0], [0], [0], [1]])
#camera calibration
    if(calibration):
        CCM = camera_calibration_matrix_2()
        column_to_be_added = np.array([[0], [0], [0]])
        CCM_inv = np.linalg.inv(CCM)
        # Adding column to array using append() method
        CCM_inv = np.append(CCM_inv, column_to_be_added, axis=1)
        newrow = np.array([0, 0, 0, 1])
        CCM_inv = np.vstack([CCM_inv, newrow])

        pix_x = 334.28368528
        pix_y = 0
#given
    #target coord in camera
    pix = np.array([[pix_x], [pix_y], [1], [1]])
    #distance of center of rotation of gimbal from centroid of PA
    g_dist = np.array([0,0,0])
    #distance of camera center vision from center of rotation of gimbal
    c_dist = np.array([0,0,0])
    f = 0.035
    #f = 650*0.0003741
    focal_lenght_meters =np.array([[f,0,0,0],[0,f,0,0],[0,0,f,0],[0,0,0,1]])
    cam = np.array([[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
    cc = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    im = np.array([[f, 0, 0, 0], [0, f, 0, 0], [0, 0, f, 0], [0, 0, 0, 1]])
#########
    trans_c = Transformation_translation(c_dist[0], c_dist[1], c_dist[2])
    rot_g = Transformation_rotation(g_yaw, g_pitch, g_roll)
    trans_g = Transformation_translation(g_dist[0], g_dist[1], g_dist[2])
    rot_v = Transformation_rotation(yaw,pitch,roll)
    trans_i = Transformation_translation(x, y, z)
    P_cc = np.linalg.multi_dot([np.linalg.inv(trans_i[0]),rot_v, np.linalg.inv(trans_g[0]),rot_g, np.linalg.inv(trans_c[0]),C])
    q_obj = np.linalg.multi_dot([np.linalg.inv(trans_i[0]),rot_v, np.linalg.inv(trans_g[0]),rot_g, np.linalg.inv(trans_c[0]), cam, im, cc,CCM_inv,pix])
    t = isect_line_plane_v3(P_cc[:3],q_obj[:3],[1,1,0],[0,0,1])

    t_norm = np.linalg.norm((t-P_cc[:3]),2)
    r = np.sqrt(np.square(pix[0][0]-CCM[0][2])+np.square(pix[1][0]-CCM[1][2]))
    Beta = np.arctan((r/((CCM[0][0]+CCM[1][1])/2))) #angle between depth and line to target from center of optical lens
    l = t_norm*np.cos(Beta)
    Q =np.array([[l,0,0,0],[0,l,0,0],[0,0,l,0],[0,0,0,1]])
    T = np.linalg.multi_dot([np.linalg.inv(trans_i[0]),rot_v, np.linalg.inv(trans_g[0]),rot_g, np.linalg.inv(trans_c[0]), cam, Q, cc,CCM_inv,])


########

    p = np.array([[1], [2], [-3], [1]])

    Q =np.array([[-l,0,0,0],[0,-l,0,0],[0,0,-l,0],[0,0,0,1]])
    In = np.dot(trans_i[0],p)
    In2 = np.linalg.multi_dot([rot_v,p])
    in3 = np.linalg.multi_dot([trans_g[0],In2])
    inertial_frame_coord =np.linalg.multi_dot([T,pix])
#results
    target_location  = inertial_frame_coord[:3]
    latitude = inertial_frame_coord[0]
    longitude = inertial_frame_coord[1]
    height = inertial_frame_coord[2]

#plots
    cam2target = t[:3] # no need
    in2body= np.dot(np.linalg.inv(np.linalg.multi_dot([rot_v,trans_i[0]])),C)
    body2gim = np.dot(np.linalg.inv(np.linalg.multi_dot([rot_g,trans_g[0],rot_v,trans_i[0]])),C)
    gim2cam = np.dot(np.linalg.inv(np.linalg.multi_dot([trans_c[0],rot_g,trans_g[0],rot_v,trans_i[0]])),C)
    fig = plt.figure()
    ax = plt.axes(projection = '3d')
    ax.set_xlim([-10,30])
    ax.set_ylim([-20,20])
    ax.set_zlim([-30,0])
    b_g = [body2gim[0][0]-in2body[0][0] ,
              body2gim[1][0]-in2body[1][0], body2gim[2][0]-in2body[2][0]]

    g_c = [-(b_g[0]+in2body[0][0])+gim2cam[0][0], -(b_g[1]+in2body[1][0])+gim2cam[1][0], -(b_g[2]+in2body[2][0])+gim2cam[2][0]]
    c_t = [t[0]-(in2body[0][0]+b_g[0]+g_c[0]), t[1]-(in2body[1][0]+b_g[1]+g_c[1]), t[2]-(in2body[2][0]+b_g[2]+g_c[2])]
    start = [0,0,0]
    ax.quiver(start[0],start[1],start[2],in2body[0][0],in2body[1][0],in2body[2][0]) #inertial to body
    ax.quiver(in2body[0][0], in2body[1][0], in2body[2][0], b_g[0], b_g[1], b_g[2], color='y') #body to gimbal
    ax.quiver(b_g[0]+in2body[0][0], b_g[1]+in2body[1][0], b_g[2]+in2body[2][0],g_c[0],g_c[1],g_c[2], color = 'g') #gimbal to cam
    ax.quiver(b_g[0]+in2body[0][0]+g_c[0], b_g[1]+in2body[1][0]+g_c[1], b_g[2]+in2body[2][0]+g_c[2],c_t[0],c_t[1],c_t[2], color = 'r') #cam to target

    p = np.array([[1], [2], [-3], [1]])
    rot_v = Transformation_rotation(yaw, pitch, roll)
    T1 = np.linalg.multi_dot([rot_v,p])


    return latitude,longitude,height
if __name__ == '__main__':
    coor_camera_to_inertial_frame (x = 0,y = 0 , z = -210,roll=0,yaw=0,pitch=-np.pi/3,g_roll=0, g_yaw=0, g_pitch=0, calibration = True)
#xyz = longitude,latitude and height of PA
#g_a_dist = distance in x y z from the centroi of PA to gimbal
#g_roll, g_yaw, g_pitch = gimbal's rotation
#follows right hand rule convention

#angles in rad
#yaw pitch roll convention

#follow convention NED (North East Down)