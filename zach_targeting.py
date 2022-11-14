import zach_math as zm
import numpy as np
import matplotlib.pyplot as plt

ac_pos = (10, 10, 10) # Aircraft position in inertial frame (x, y, h)
ac_att = (0, 0, 0) # Aircraft attitude in inertial frame (pitch, yaw, roll)
gim_pos = (0, 0, 0) # Gimbal position in body frame (x, y, h)
gim_att = (-1*np.pi/3, 0, 0) # Gimbal attitude in body frame (pitch, yaw, roll)
cam_pos = (0, 0, 0) # Camera position in gimbal frame (x, y, h)
pos_c = (0, 0, 10) # Position of object in camera frame (x, y, z)

# Position of object in inertial frame (x, y, h)
pos_i = zm.transform_camera_to_inertial(ac_pos, ac_att, gim_pos, gim_att, cam_pos, pos_c)

print(f"x {pos_i[0]}, y {pos_i[1]}, z {pos_i[2]}")

fig = plt.figure()
ax = plt.axes(projection = '3d')
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([0, 10])

ax.quiver(0, 0, 0, ac_pos[0], ac_pos[1], ac_pos[2], color='orange')
ax.quiver(0, 0, 0, pos_i[0], pos_i[1], -1*pos_i[2], color='blue')

plt.show()