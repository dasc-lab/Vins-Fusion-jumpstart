import rosbag
import numpy as np
import matplotlib.pyplot as plt

bag = rosbag.Bag('v1.bag', 'r')

traj_vicon = []
traj_vins  = []

for topic, msg, t in bag.read_messages(topics=["/tf"]):
    transform = msg.transforms[0]
    if transform.header.frame_id == "/vicon/world":
        traj_vicon.append(transform)
    elif transform.header.frame_id == "world":
        traj_vins.append(transform)


## plot the trajectory
traj_vicon_t = []
traj_vicon_x = []
traj_vicon_y = []
traj_vicon_z = []

traj_vins_t = []
traj_vins_x = []
traj_vins_y = []
traj_vins_z = []

for tf in traj_vicon:
    traj_vicon_t.append(tf.header.stamp.to_sec())
    traj_vicon_x.append(tf.transform.translation.x)
    traj_vicon_y.append(tf.transform.translation.y)
    traj_vicon_z.append(tf.transform.translation.z)

for tf in traj_vins:
    traj_vins_t.append(tf.header.stamp.to_sec())
    traj_vins_x.append(tf.transform.translation.x)
    traj_vins_y.append(tf.transform.translation.y)
    traj_vins_z.append(tf.transform.translation.z)

## interpolate vicon trajectory to match vins times
traj_vicon_x_interp = np.interp(traj_vins_t, traj_vicon_t, traj_vicon_x)
traj_vicon_y_interp = np.interp(traj_vins_t, traj_vicon_t, traj_vicon_y)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

ax1.plot(traj_vicon_x, traj_vicon_y, label='Vicon')
#ax1.plot(traj_vins_x, traj_vins_y, label='VINS')
ax1.plot(traj_vins_y,  [-v for v in traj_vins_x], label='VINS')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('Trajectory Comparison')
ax1.legend()

## calculate error between the two trajectories
error_x = traj_vins_y - traj_vicon_x_interp
error_y = [-v for v in traj_vins_x] - traj_vicon_y_interp
error = np.sqrt(error_x**2 + error_y**2)

ax2.plot(traj_vins_t, error, label='Error')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error')
ax2.set_title('Trajectory Error (2D)')
ax2.legend()

plt.show()

