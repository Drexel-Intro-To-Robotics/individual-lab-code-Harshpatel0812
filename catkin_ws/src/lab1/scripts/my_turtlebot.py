#!/usr/bin/env python3

import rospy
import math
import sys
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
import tf.transformations


class myTurtle():

    def __init__(self) -> None:
        """
        Initialize the ROS node, publisher, and subscriber.
        """
        rospy.init_node('my_turtle_node', anonymous=True)

        # Publisher: sends velocity commands to the robot
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber: listens to odometry so we know the robot's position
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        # Subscriber: listens for nav goal from RViz or command line
        self.goal_sub = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.goal_callback
        )

        # TurtleBot3 Waffle physical constants
        self.wheel_radius = 0.033   # meters
        self.wheel_base   = 0.287   # meters (distance between wheels)

        # Internal position state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        # Control loop rate (10 Hz)
        self.rate = rospy.Rate(10)

        rospy.loginfo("myTurtle node initialized!")

    # ------------------------------------------------------------------

    def odom_cb(self, msg: Odometry) -> None:
        """
        Odometry callback — updates internal x, y, yaw from /odom topic.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        self.yaw = self.convert_to_euler(quat)

        self.odom_received = True

    # ------------------------------------------------------------------

    def goal_callback(self, msg: PoseStamped) -> None:
        """
        Nav goal callback — triggered when a goal is published to
        /move_base_simple/goal (from RViz 2D Nav Goal or command line).
        """
        rospy.loginfo("Received nav goal!")
        self.nav_to_pose(msg)

    # ------------------------------------------------------------------

    def convert_to_euler(self, quat) -> float:
        """
        Convert a quaternion to a yaw (heading) angle in radians.
        """
        q = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(q)
        return yaw

    # ------------------------------------------------------------------

    def stop(self) -> None:
        """
        Stop the robot by publishing zero velocity.
        """
        twist = Twist()   # all zeros by default
        self.cmd_pub.publish(twist)
        rospy.loginfo("Robot stopped.")

    # ------------------------------------------------------------------

    def spin_wheels(self, u1, u2, time):
        """
        Spin the two wheels at given speeds for a given time.
        u1: left wheel speed  (rad/s)
        u2: right wheel speed (rad/s)
        time: duration in seconds
        """
        twist = Twist()
        # Convert wheel speeds to linear and angular velocity
        v = self.wheel_radius * (u1 + u2) / 2.0
        w = self.wheel_radius * (u2 - u1) / self.wheel_base

        twist.linear.x  = v
        twist.angular.z = w

        end_time = rospy.Time.now() + rospy.Duration(time)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            self.rate.sleep()

        self.stop()

    # ------------------------------------------------------------------

    def drive_straight(self, dist: float, vel: float = 0.2) -> None:
        """
        Drive straight for 'dist' meters at 'vel' m/s.
        Uses odometry to know when we've gone far enough.
        """
        if not self.odom_received:
            rospy.logwarn("No odometry yet — cannot drive straight.")
            return

        start_x = self.x
        start_y = self.y

        twist = Twist()
        twist.linear.x = abs(vel) if dist >= 0 else -abs(vel)

        rospy.loginfo(f"Driving straight {dist:.2f} m at {vel:.2f} m/s")

        while not rospy.is_shutdown():
            dx = self.x - start_x
            dy = self.y - start_y
            traveled = math.sqrt(dx**2 + dy**2)

            if traveled >= abs(dist):
                break

            self.cmd_pub.publish(twist)
            self.rate.sleep()

        self.stop()

    # ------------------------------------------------------------------

    def rotate(self, angle: float, ang_vel: float = 0.5) -> None:
        """
        Rotate in place by 'angle' radians.
        Positive = counter-clockwise, Negative = clockwise.
        """
        if not self.odom_received:
            rospy.logwarn("No odometry yet — cannot rotate.")
            return

        start_yaw = self.yaw
        target_yaw = start_yaw + angle

        twist = Twist()
        twist.angular.z = ang_vel if angle >= 0 else -ang_vel

        rospy.loginfo(f"Rotating {math.degrees(angle):.1f} degrees")

        rotated = 0.0
        prev_yaw = start_yaw

        while not rospy.is_shutdown():
            delta = self.yaw - prev_yaw

            # Handle angle wrap-around (e.g., from π to -π)
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi

            rotated  += delta
            prev_yaw  = self.yaw

            if abs(rotated) >= abs(angle):
                break

            self.cmd_pub.publish(twist)
            self.rate.sleep()

        self.stop()

    # ------------------------------------------------------------------

    def drive_circle(self, radius: float, vel: float = 0.2) -> None:
        """
        Drive in a circle of given radius (meters).
        Completes one full circle (2π radians).
        """
        circumference = 2 * math.pi * radius
        angular_vel   = vel / radius

        twist = Twist()
        twist.linear.x  = vel
        twist.angular.z = angular_vel

        # Time to complete one full circle
        duration = circumference / vel
        end_time = rospy.Time.now() + rospy.Duration(duration)

        rospy.loginfo(f"Driving circle: radius={radius}m, duration={duration:.1f}s")

        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            self.rate.sleep()

        self.stop()

    # ------------------------------------------------------------------

    def drive_square(self, side: float) -> None:
        rospy.loginfo(f"Driving square: side={side}m")
        for i in range(4):
            rospy.loginfo(f"  Side {i+1} of 4")
            self.drive_straight(side)
            rospy.sleep(0.5)
            self.rotate(math.pi / 2)
            rospy.sleep(0.5)
        rospy.loginfo("Square complete!")

    # ------------------------------------------------------------------

    def random_dance(self) -> None:
        rospy.loginfo("Starting random dance!")

        # Move 1: drive forward
        self.drive_straight(0.3)
        rospy.sleep(0.3)

        # Move 2: spin 180
        self.rotate(math.pi)
        rospy.sleep(0.3)

        # Move 3: drive forward again
        self.drive_straight(0.3)
        rospy.sleep(0.3)

        # Move 4: small circle
        self.drive_circle(0.2)
        rospy.sleep(0.3)

        # Move 5: spin the other way
        self.rotate(-math.pi)
        rospy.sleep(0.3)

        # Move 6: zigzag
        self.drive_straight(0.2)
        self.rotate(math.pi / 4)
        self.drive_straight(0.2)
        self.rotate(-math.pi / 2)
        self.drive_straight(0.2)
        rospy.sleep(0.3)

        # Move 7: fast spin in place using spin_wheels
        self.spin_wheels(-5.0, 5.0, 2.0)
        rospy.sleep(0.3)

        # Move 8: drive back to roughly start
        self.rotate(math.pi / 2)
        self.drive_straight(0.3)

        rospy.loginfo("Dance complete!")

    # ------------------------------------------------------------------

    def nav_to_pose(self, goal: PoseStamped) -> None:
        """
        Drive to a goal pose: first drive straight to (x,y), then rotate to match theta.
        """
        goal_x   = goal.pose.position.x
        goal_y   = goal.pose.position.y
        goal_yaw = self.convert_to_euler(goal.pose.orientation)

        # --- 1. Rotate to face the goal ---
        dx    = goal_x - self.x
        dy    = goal_y - self.y
        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_to_goal - self.yaw

        # Normalize to [-π, π]
        while heading_error >  math.pi: heading_error -= 2 * math.pi
        while heading_error < -math.pi: heading_error += 2 * math.pi

        self.rotate(heading_error)
        dist = math.sqrt(dx**2 + dy**2)
        self.drive_straight(dist)

        final_error = goal_yaw - self.yaw
        while final_error >  math.pi: final_error -= 2 * math.pi
        while final_error < -math.pi: final_error += 2 * math.pi

        self.rotate(final_error)

        rospy.loginfo("Reached goal pose!")


def print_usage():
    print("")
    print("=" * 50)
    print("  Usage: rosrun lab1 my_turtlebot.py <task>")
    print("=" * 50)
    print("  Tasks:")
    print("    circle  → Task 5: Drive in a 0.5m circle")
    print("    square  → Task 6: Drive in a 0.5m square")
    print("    dance   → Task 8: Random dance moves")
    print("    nav     → Task 7: Wait for /move_base_simple/goal")
    print("=" * 50)
    print("  Example:")
    print("    rosrun lab1 my_turtlebot.py nav")
    print("=" * 50)
    print("")


# ======================================================================

def main():
    # Read the task argument passed by the user
    # sys.argv[0] is the script name, sys.argv[1] is the task
    args = rospy.myargv(argv=sys.argv)

    if len(args) < 2:
        print_usage()
        sys.exit(1)

    task = args[1].lower().strip()

    if task not in ['circle', 'square', 'dance', 'nav']:
        print(f"\n Unknown task: '{task}'")
        print_usage()
        sys.exit(1)

    # Create the robot
    turtle = myTurtle()
    rospy.sleep(1)  # Wait for odometry to start

    # Run ONLY the requested task
    if task == 'circle':
        rospy.loginfo("=== Task 5: Circle (r=0.5m) ===")
        turtle.drive_circle(0.5)
        rospy.loginfo("=== Circle done! ===")

    elif task == 'square':
        rospy.loginfo("=== Task 6: Square (side=0.5m) ===")
        turtle.drive_square(0.5)
        rospy.loginfo("=== Square done! ===")

    elif task == 'dance':
        rospy.loginfo("=== Task 8: Random Dance ===")
        turtle.random_dance()
        rospy.loginfo("=== Dance done! ===")

    elif task == 'nav':
        rospy.loginfo("=== Task 7: Waiting for nav goal on /move_base_simple/goal ===")
        rospy.loginfo("    Publish a goal using rostopic pub or RViz 2D Nav Goal button")
        rospy.spin()  # keeps node alive, goal_callback fires when goal arrives


if __name__ == '__main__':
    main()