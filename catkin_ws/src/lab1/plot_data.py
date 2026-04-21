#!/usr/bin/env python3
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

def extract_bag(bag_file):
    odom_t, odom_x, odom_y, odom_lx, odom_az = [], [], [], [], []
    imu_t, imu_ax, imu_ay, imu_az, imu_wz    = [], [], [], [], []
    cmd_t, cmd_lx, cmd_az                     = [], [], []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(
                topics=['/odom', '/imu', '/cmd_vel']):
            ts = t.to_sec()
            if topic == '/odom':
                odom_t.append(ts)
                odom_x.append(msg.pose.pose.position.x)
                odom_y.append(msg.pose.pose.position.y)
                odom_lx.append(msg.twist.twist.linear.x)
                odom_az.append(msg.twist.twist.angular.z)
            elif topic == '/imu':
                imu_t.append(ts)
                imu_ax.append(msg.linear_acceleration.x)
                imu_ay.append(msg.linear_acceleration.y)
                imu_az.append(msg.linear_acceleration.z)
                imu_wz.append(msg.angular_velocity.z)
            elif topic == '/cmd_vel':
                cmd_t.append(ts)
                cmd_lx.append(msg.linear.x)
                cmd_az.append(msg.angular.z)

    return (odom_t, odom_x, odom_y, odom_lx, odom_az,
            imu_t,  imu_ax, imu_ay, imu_az,  imu_wz,
            cmd_t,  cmd_lx, cmd_az)

def rel(times):
    if not times:
        return []
    t0 = times[0]
    return [t - t0 for t in times]

def plot_all(bag_file, title):
    if not os.path.exists(bag_file):
        print(f"[SKIP] {bag_file} not found — record it first.")
        return

    print(f"Reading {bag_file} ...")
    (odom_t, odom_x, odom_y, odom_lx, odom_az,
     imu_t,  imu_ax, imu_ay, imu_az,  imu_wz,
     cmd_t,  cmd_lx, cmd_az) = extract_bag(bag_file)

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(title, fontsize=14, fontweight='bold')

    # Odom XY path
    ax = axes[0, 0]
    if odom_x:
        ax.plot(odom_x, odom_y, color='steelblue', linewidth=1.5)
        ax.scatter([odom_x[0]],  [odom_y[0]],  color='green', zorder=5, label='start')
        ax.scatter([odom_x[-1]], [odom_y[-1]], color='red',   zorder=5, label='end')
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.set_aspect('equal'); ax.legend(fontsize=8)
    ax.set_title('Odom path (x vs y)'); ax.grid(True, alpha=0.3)

    # Odom position vs time
    ax = axes[0, 1]
    if odom_t:
        t = rel(odom_t)
        ax.plot(t, odom_x, label='x (m)')
        ax.plot(t, odom_y, label='y (m)')
        ax.set_xlabel('time (s)'); ax.set_ylabel('position (m)')
        ax.legend(fontsize=8)
    ax.set_title('Odom position vs time'); ax.grid(True, alpha=0.3)

    # IMU linear acceleration
    ax = axes[1, 0]
    if imu_t:
        t = rel(imu_t)
        ax.plot(t, imu_ax, label='ax')
        ax.plot(t, imu_ay, label='ay')
        ax.plot(t, imu_az, label='az', alpha=0.5)
        ax.set_xlabel('time (s)'); ax.set_ylabel('m/s2')
        ax.legend(fontsize=8)
    ax.set_title('IMU linear acceleration'); ax.grid(True, alpha=0.3)

    # IMU angular velocity
    ax = axes[1, 1]
    if imu_t:
        t = rel(imu_t)
        ax.plot(t, imu_wz, color='coral', label='wz (rad/s)')
        ax.set_xlabel('time (s)'); ax.set_ylabel('rad/s')
        ax.legend(fontsize=8)
    ax.set_title('IMU angular velocity (z)'); ax.grid(True, alpha=0.3)

    # Commanded linear velocity
    ax = axes[2, 0]
    if cmd_t:
        t = rel(cmd_t)
        ax.plot(t, cmd_lx, color='green', label='linear.x (m/s)')
        ax.set_xlabel('time (s)'); ax.set_ylabel('m/s')
        ax.legend(fontsize=8)
    ax.set_title('Commanded linear velocity'); ax.grid(True, alpha=0.3)

    # Commanded angular velocity
    ax = axes[2, 1]
    if cmd_t:
        t = rel(cmd_t)
        ax.plot(t, cmd_az, color='purple', label='angular.z (rad/s)')
        ax.set_xlabel('time (s)'); ax.set_ylabel('rad/s')
        ax.legend(fontsize=8)
    ax.set_title('Commanded angular velocity'); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = title.replace(' ', '_') + '.png'
    plt.savefig(out, dpi=150)
    print(f"  Saved -> {out}")
    plt.close()

# --- bag files are in catkin_ws/ so use full path ---
base = '/workspaces/individual-lab-code-Harshpatel0812/catkin_ws/'
plot_all(base + 'circle_bag.bag',   'Circle Task')
plot_all(base + 'square_bag.bag',   'Square Task')
plot_all(base + 'dance_bag.bag',    'Dance Task')
plot_all(base + 'nav_goal_bag.bag', 'Nav Goal Task')
print("\nDone! PNG files saved in catkin_ws/src/lab1/")
