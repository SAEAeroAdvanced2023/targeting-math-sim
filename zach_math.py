import numpy as np

def C(x):
    return np.cos(x)

def S(x):
    return np.sin(x)

def transform(x, y, z, pitch, yaw, roll):
    matrix = np.array( [[C(pitch)*C(yaw),                           C(pitch)*S(yaw),                            -1*S(pitch),        x],
                        [S(roll)*S(pitch)*C(yaw) - C(roll)*S(yaw),  S(roll)*S(pitch)*S(yaw) + C(roll)*C(yaw),   S(roll)*C(pitch),   y],
                        [C(roll)*S(pitch)*C(yaw) + S(roll)*S(yaw),  C(roll)*S(pitch)*S(yaw) - S(roll)*C(yaw),   C(roll)*C(pitch),   z],
                        [0,                                         0,                                          0,                  1]])
    return matrix

def transform_camera_to_inertial(ac_pos, ac_att, gim_pos, gim_att, cam_pos, pos_c):
    ac_x = ac_pos[0]
    ac_y = ac_pos[1]
    ac_h = ac_pos[2]
    ac_pitch = ac_att[0]
    ac_yaw = ac_att[1]
    ac_roll = ac_att[2]
    
    T_i_to_b = transform(-1*ac_x, -1*ac_y, ac_h, ac_pitch, ac_yaw, ac_roll)

    gim_x = gim_pos[0]
    gim_y = gim_pos[1]
    gim_h = gim_pos[2]
    gim_pitch = gim_att[0]
    gim_yaw = gim_att[1]
    gim_roll = gim_att[2]

    T_b_to_g = transform(-1*gim_x, -1*gim_y, gim_h, gim_pitch, gim_yaw, gim_roll)

    cam_x = cam_pos[0]
    cam_y = cam_pos[1]
    cam_h = cam_pos[2]
    cam_pitch = np.pi/2
    cam_yaw = 0
    cam_roll = 0

    T_g_to_c = transform(-1*cam_x, -1*cam_y, cam_h, cam_pitch, cam_yaw, cam_roll)

    #  T_g_to_c * T_b_to_g * T_i_to_b

    T_i_to_c = np.matmul(np.matmul(T_g_to_c, T_b_to_g), T_i_to_b)
    T_c_to_i = np.linalg.inv(T_i_to_c)

    pos_c_x = pos_c[0]
    pos_c_y = pos_c[1]
    pos_c_z = pos_c[2]

    pos_c_4 = np.array([[pos_c_x], [pos_c_y], [pos_c_z], [1]])
    pos_i_4 = np.matmul(T_c_to_i, pos_c_4)

    pos_i = (pos_i_4[0][0], pos_i_4[1][0], pos_i_4[2][0])

    return pos_i