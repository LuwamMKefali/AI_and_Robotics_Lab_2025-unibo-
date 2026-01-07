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

from atlantis_msgs.srv import DoorCommand
from atlantis_msgs.srv import DoorStatus
from atlantis_msgs.srv import BoxPos
from atlantis_msgs.srv import BoxPickUp
from atlantis_msgs.srv import BoxPutDown

node = None

robot_subs = None
sonar_subs = []
encoders = None
realpose = None
vel_publisher_node = None

cmd_vel_publisher = None
doors_clients = None
boxes_pos_clients = None
boxes_pick_clients = None
boxes_drop_clients = None
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
    global doors_clients, boxes_pos_clients, boxes_pick_clients, boxes_drop_clients
    global node, vel_publisher_node, executor
    
    node = rclpy.create_node('robot_client_nodes')
    vel_publisher_node = rclpy.create_node('vel_publisher_node')
    cmd_vel_publisher = vel_publisher_node.create_publisher(Twist, '/mobile_base_controller/cmd_vel_unstamped', 1000)
 
    doors_clients = {
            'door1': node.create_client(DoorCommand, 'door1/door_command'),
            'door2': node.create_client(DoorCommand, 'door2/door_command'),
            'door3': node.create_client(DoorCommand, 'door3/door_command'),
            'door4': node.create_client(DoorCommand, 'door4/door_command'),
    }
    boxes_pos_clients = {
            'BOX1': node.create_client(BoxPos, 'BOX1/get_position'),
            'BOX2': node.create_client(BoxPos, 'BOX2/get_position'),
            'BOX3': node.create_client(BoxPos, 'BOX3/get_position')
    }
    boxes_pick_clients = {
            'BOX1': node.create_client(BoxPickUp, 'BOX1/pick_up'),
            'BOX2': node.create_client(BoxPickUp, 'BOX2/pick_up'),
            'BOX3': node.create_client(BoxPickUp, 'BOX3/pick_up')
    }
    boxes_drop_clients = {
            'BOX1': node.create_client(BoxPutDown, 'BOX1/put_down'),
            'BOX2': node.create_client(BoxPutDown, 'BOX2/put_down'),
            'BOX3': node.create_client(BoxPutDown, 'BOX3/put_down')
    }
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

    
def get_box_position (box):
    """
    Virtual sensor to read the current (x,y) position of a given box in global frame
    In simulation, this uses a ad-hoc service implemented in Gazebo
    In a real physical setup, this could be implemented by a set of cameras in the environments
    """
    node.get_logger().info('Requested Position of %s' % (box))
    req = BoxPos.Request()
    req.request = True
    client = boxes_pos_clients.get(box.upper())
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    node.get_logger().info(
        'Position of %s: (%f, %f, %f, %f)' % (box, response.x_pos, response.y_pos, response.z_pos, response.yaw))
    return (response.x_pos, response.y_pos, response.yaw, 0.3)  # last value is box radius

   
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


def execute_door_command (door, command):        # door is a door name, command is "Open" or "Close"
    req = DoorCommand.Request()
    node.get_logger().info('Request to %s %s!' % (command, door))
    req.command = command
    client = doors_clients.get(door)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()

    if (response.done):
        node.get_logger().info('%s %s!' % (command, door))
    else:
        node.get_logger().error('Failed to %s %s!' % (command, door))
    

def pick_up_box (box):                          # box is a box name
    node.get_logger().info('Picking up box %s' % (box))
    req = BoxPickUp.Request()
    req.action = True
    client = boxes_pick_clients.get(box.upper())
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if(response.success):
        node.get_logger().info('Picked up %s ' % (box))
    else:
        node.get_logger().error('Failed to pick up %s ' % (box))
    return response.success


def put_down_box (box):
    node.get_logger().info('Putting down box %s' % (box))
    req = BoxPutDown.Request()
    req.action = True
    client = boxes_drop_clients.get(box.upper())
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if(response.success):
        node.get_logger().info('Put down %s ' % (box))
    else:
        node.get_logger().error('Failed to put down %s ' % (box))
    return response.success


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
