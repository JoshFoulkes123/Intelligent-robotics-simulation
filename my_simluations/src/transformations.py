#!/usr/bin/env python3

from math import sqrt, cos, sin, atan2, degrees, asin
from geometry_msgs.msg import Quaternion

MAP_WIDTH = 384
MAP_HEIGHT = 864
LASER_MAX = 8.0
RESOLUTION = 0.05
CENTER_X = 230
CENTER_Y = 340

def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = degrees(atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = degrees(asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = degrees(atan2(t3, t4))

    return X, Y, Z

def world_to_pixel(world_points, image_size):
    world_x, world_y = world_points
    img_h, img_w = image_size
    pixel_points = []
    pixel_points[0] = int(max((world_x / MAP_WIDTH) * img_w, 0))
    if pixel_points[0] > img_w - 1:
        pixel_points[0] = img_w - 1
    pixel_points[1] = int(max((world_y / MAP_HEIGHT) * img_h, 0))
    if pixel_points[1] > img_h - 1:
        pixel_points[1] = img_h
    pixel_points[1] = pixel_points[1]
    pixel_points[0] = img_w/2 + pixel_points[0]
    pixel_points[1] = img_h/2 - pixel_points[1]
    return pixel_points


def pixel_to_world(pixel_points, image_size):
    img_h, img_w = image_size
    pixel_x, pixel_y = pixel_points
    world_points = []
    world_points[0] = pixel_x/img_w * MAP_WIDTH
    world_points[1] = (pixel_y/img_h * MAP_HEIGHT)
    world_points[0] = world_points[0] - MAP_WIDTH/2
    world_points[1] = world_points[0] + MAP_HEIGHT/2
    return world_points

def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.

    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw

def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.

    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0

    sinp = sin(p)
    siny = sin(y)
    sinr = sin(r)
    cosp = cos(p)
    cosy = cos(y)
    cosr = cos(r)

    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    #rospy.loginfo(q_headingChange)##edited

    # ----- Multiply new (heading-only) quaternion by the existing (pitch and bank)
    # ----- quaternion. Order is important! Original orientation is the second
    # ----- argument rotation which will be applied to the quaternion is the first
    # ----- argument.
    return multiply_quaternions(q_headingChange, q_orig)


def multiply_quaternions( qa, qb ):
    """
    Multiplies two quaternions to give the rotation of qb by qa.

    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()

    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    # rospy.loginfo(combined)##edited

    return combined


def worldtheta_to_pixeltheta(world_theta):
    x = 1 * cos(world_theta)
    y = 1 * sin(world_theta)
    # since the y axis is inverted in the map the angle will be different
    return atan2(-y, x)

def pixel_to_real(_x, _y):
    x = _x*RESOLUTION
    y = (MAP_HEIGHT-_y)*RESOLUTION
    return x,y

def real_to_pixel(_x,_y):
    x = int(_x/RESOLUTION)
    y = MAP_HEIGHT-int(_y/RESOLUTION)
    return x,y


def dist(point_a, point_b):
    return sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)
