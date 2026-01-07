"""
============== UniBo: AI and Robotics 2025 ==============
Base code: gateway to the robot (or simulator)
This is a base version, it needs to be completed by the students with new
subscribers, publishers or services needed for the different labs

(c) 2024-2025 Alessandro Saffiotti
"""

import math, time
import numpy as np

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Range
from nav_msgs.msg import Odometry


node = None
robot_subs = None
sonar_subs = []
vel_publisher_node = None
cmd_vel_publisher = None

encoders = None
ground_truth_pose = None  # a global variable for ground truth
sonars = [0.0] * 12


class Quaternion:
    """
    ROS measures angles in quaternions, this class provides a convenience
    function to convert quaternions into Euler angles
    """
    def __init__ (self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def to_euler (self):
        x = self.x
        y = self.y
        z = self.z
        w = self.w

        # Normalize quaternion to avoid errors
        norm = np.sqrt(x**2 + y**2 + z**2 + w**2)
        x /= norm
        y /= norm
        z /= norm
        w /= norm

        # Roll
        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

        # Pitch
        sin_pitch = 2.0 * (w * y - z * x)
        sin_pitch = np.clip(sin_pitch, -1.0, 1.0) 
        pitch = np.arcsin(sin_pitch)

        # Yaw
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return roll, pitch, yaw


def sonar_ring_pose (bearing):
    """
    This is a convenience function to compute the pose of each sonar sensor in the robot base
    """
    rho = 0.260
    phi = math.radians(bearing)
    x = rho * math.cos(phi)
    y = rho * math.sin(phi)
    return (x, y, phi)

parameters = {                        # robot's parameters, these are for the Tiago base
    'wheel_radius' : 0.0985,
    'wheel_axis' : 0.4044,
    'sonar_num' : 12,
    'sonar_maxrange' : 3.0,
    'sonar_delta' : math.radians(25.0),
    'sonar_poses' : [sonar_ring_pose(0.0),
                     sonar_ring_pose(30.0),
                     sonar_ring_pose(60.0),
                     sonar_ring_pose(90.0),
                     sonar_ring_pose(120.0),
                     sonar_ring_pose(150.0),
                     sonar_ring_pose(180.0),
                     sonar_ring_pose(210.0),
                     sonar_ring_pose(240.0),
                     sonar_ring_pose(270.0),
                     sonar_ring_pose(300.0),
                     sonar_ring_pose(330.0)]
}


def encoders_callback (msg):
    """
    Called when a new wheel encoder message is received
    """
    global encoders
    encoders = (msg.position[1], msg.position[0])
    #node.get_logger().info('Encoders: "%f, %f"' % (msg.position[12], msg.position[13]))


def sonar_callback (i):
    """
    Called when a new sonar range message is received
    """
    def cb(msg):
        global sonars
        sonars[i] = msg.range
        #robot_subs.get_logger().info('Range: %f' % (msg.range))
    return cb

def ground_truth_callback (msg) :
    """
    Called when a new groudn truth odometry message is received
    """
    global ground_truth_pose
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    q = Quaternion(ori.x, ori.y, ori.z, ori.w)
    _, _, yaw = q.to_euler()
    ground_truth_pose = (pos.x, pos.y, yaw)



def is_ready ():
    """
    Check if ROS has already started publishing messages
    """
    executor.spin_once(timeout_sec=0.01) 
    return encoders != None


def robot_alive ():
    """
    Check if ROS is still running
    """
    return rclpy.ok()


def init_ros ():
    """
    Initialize ROS
    """
    rclpy.init()
    print("ROS initialized")


def startup_robot ():
    """
    Set up the communication channels with the robot (or simulator)
    and perform any initialization needed
    """
    global parameters
    global cmd_vel_publisher, robot_subs
    global node, vel_publisher_node, executor
    
    node = rclpy.create_node('robot_client_nodes')
    vel_publisher_node = rclpy.create_node('vel_publisher_node')
    cmd_vel_publisher = vel_publisher_node.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 1000)
 
    robot_subs = init_subs()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot_subs)
    executor.add_node(vel_publisher_node)
    print("Robot started")
    return parameters


def init_subs ():
    """
    Create subscriptions to all the relevant topics, and link them
    to the corresponding callback functions.  You may need to extend
    this if you want to subscribe to additional topics.
    """
    global robot_subs, enc_sub, executor
    robot_subs = rclpy.create_node('robot_subs_nodes')
    enc_sub = robot_subs.create_subscription(
            JointState,
            '/joint_states',
            encoders_callback,
            1)    
    gt_sub = robot_subs.create_subscription(
            Odometry,
            '/ground_truth_odom',
            ground_truth_callback,
            1)
    for i in range(parameters['sonar_num']):
        sonar_sub = robot_subs.create_subscription(
                Range,
                '/sonar{:02d}_base'.format(i+1),
                sonar_callback(i),
                1)
        sonar_subs.append(sonar_sub)
    return robot_subs


def get_sonar_data ():
    """
    Get the latest received values from the sonar ring
    Returns an array of readings in the form (sonar pose, range)
    Sonar pose is (x, y, th) in robot's base frame
    """
    duration = 0.01                                     # spin for this duration
    start_time = time.time()
    executor.add_node(robot_subs)
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.01) 
        if time.time() - start_time > duration:         # duration exceeded
            break
    res = []
    for i in range(parameters['sonar_num']):
        res.append((parameters['sonar_poses'][i], sonars[i]))
        #node.get_logger().info('Data %f' % (sonars[i]))
    return res


def get_wheel_encoders ():
    """
    Get the latest received values of wheel encoders, which give the current
    position of each wheel, in radiants
    """
    global encoders
    duration = 0.01                                     # spin for this duration
    start_time = time.time()
    executor.add_node(robot_subs)
    while rclpy.ok():
        executor.spin_once(timeout_sec=0.01) 
        if time.time() - start_time > duration:         # duration exceeded
            break
    if encoders == None:
        return (0.0, 0.0)
    return encoders

    

def get_ground_truth_pose ():
    """
    Get the latest received ground truth pose (x, y, th)
    returns None if it is not available yet
    """
    global ground_truth_pose
    executor.spin_once(timeout_sec=0.01)
    return ground_truth_pose

def set_vel_values (vlin, vrot):
    """
    Set the new linear and rotational velocities for the robot's base
    vlin is m/sec, vrot is rad/sec
    Returns True if successful
    """
    msg = Twist()
    msg.linear.x = vlin
    msg.angular.z = vrot
    cmd_vel_publisher.publish(msg)
    return True


def shutdown_robot ():
    """
    This should perform any finalization needed on the robot,
    and close all the communication channels.
    """
    node.destroy_node()
    robot_subs.destroy_node()
    vel_publisher_node.destroy_node()
    rclpy.shutdown()
    print("Robot shut")