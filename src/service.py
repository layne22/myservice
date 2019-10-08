#!/usr/bin/env python

import roslib; roslib.load_manifest('myservice')
import rospy

# from myservice.srv import WordCount,WordCountResponse
from myservice.srv import trans_count,trans_countResponse

import sys
import numpy as np

import actionlib
import myservice.msg
import std_msgs.msg
import geometry_msgs.msg


import math
import argparse
import csv

""" Global variable """
arm_joint_number = 0
finger_number = 0
# prefix = 'NO_ROBOT_TYPE_DEFINED_'
prefix = 'm1n6s300_'
poses = []
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734,
                           1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq

def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_


def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_


def EulerXYZ2Matrix4x4():
    global currentCartesianCommand
    pos = currentCartesianCommand[:3]
    tx = currentCartesianCommand[3]
    ty = currentCartesianCommand[4]
    tz = currentCartesianCommand[5]
    sx = math.sin(tx)
    cx = math.cos(tx)
    sy = math.sin(ty)
    cy = math.cos(ty)
    sz = math.sin(tz)
    cz = math.cos(tz)
    rx = np.array([[1, 0, 0, 0],
                   [0, cx, -sx, 0],
                   [0, sx, cx, 0],
                   [0, 0, 0, 1]])
    ry = np.array([[cy, 0, sy, 0],
                   [0, 1, 0, 0],
                   [-sy, 0, cy, 0],
                   [0, 0, 0, 1]])
    rz = np.array([[cz, -sz, 0, 0],
                   [sz, cz, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    rot_mat = rz.dot(ry.dot(rx))
    rot_mat[0, 3] = pos[0]
    rot_mat[1, 3] = pos[1]
    rot_mat[2, 3] = pos[2]
    print(rot_mat)
    rot_m = np.array([cz * cy, -sz * cy, sy, pos[0],
                      cz * sy * sx + sz * cx, -sz * sy * sx + cz * cx, -cy * sx, pos[1],
                      -cz * sy * cx + sz * sx, sz * sy * cx + cz * sx, cy * cx, pos[2],
                      0, 0, 0, 1])
    rot_m = np.reshape(rot_m, (4, 4))
    print(rot_m)
    return rot_m


def getcurrentCartesianCommand(prefix_):
    # wait to get current position
    topic_address = '/' + prefix_ + 'driver/out/cartesian_command'
    rospy.Subscriber(topic_address, myservice.msg.KinovaPose, setcurrentCartesianCommand)
    rospy.wait_for_message(topic_address, myservice.msg.KinovaPose)
    print 'position listener obtained message for Cartesian pose. '


def setcurrentCartesianCommand(feedback):
    global currentCartesianCommand

    currentCartesianCommand_str_list = str(feedback).split("\n")

    for index in range(0, len(currentCartesianCommand_str_list)):
        temp_str=currentCartesianCommand_str_list[index].split(": ")
        currentCartesianCommand[index] = float(temp_str[1])
    # the following directly reading only read once and didn't update the value.
    # currentCartesianCommand = [feedback.X, feedback.Y, feedback.Z, feedback.ThetaX, feedback.ThetaY, feedback.Z]
    # print 'currentCartesianCommand in setcurrentCartesianCommand is: ', currentCartesianCommand

# save as mrad
def save_vect(request):
    global currentCartesianCommand
    count = request.num
    print("------>", currentCartesianCommand)

    getcurrentCartesianCommand(prefix)
    pose_file_name = 'poses/poses%02d.txt' % count
    # pose_mq, pose_mdeg, pose_mrad = unitParser(args.unit, args.pose_value, args.relative)

    try:
        # poses = [float(n) for n in pose_mq]
        vect = [count] * 7
        vect[1:] = currentCartesianCommand[:]
        print(vect)

        # result = cartesian_pose_client(poses[:3], poses[3:])
        # transformMatrix = EulerXYZ2Matrix4x4()
        # Save the array back to the file
        poses.append(vect)
        # np.savetxt(pose_file_name, trans)

        print('Cartesian pose sent!')

        a = 10
        b = 1
        return trans_countResponse(a, b)

    except rospy.ROSInterruptException:
        print "program interrupted before completion"


# save as matrix
def save_trans(request):
    global poses
    count = request.num
    # print "--------------------------"
    pose_file_name = 'poses/poses%02d.txt' % count
    # pose_mq, pose_mdeg, pose_mrad = unitParser(args.unit, args.pose_value, args.relative)

    try:

        # poses = [float(n) for n in pose_mq]
        getcurrentCartesianCommand(prefix)
        rospy.sleep(0.1)
        # result = cartesian_pose_client(poses[:3], poses[3:])
        transformMatrix = EulerXYZ2Matrix4x4()
        # Save the array back to the file
        # poses.append(transformMatrix)
        np.savetxt(pose_file_name, transformMatrix)

        print('Cartesian pose sent!')

        a = 10
        b = 1
        return trans_countResponse(a, b)

    except rospy.ROSInterruptException:
        print "program interrupted before completion"


if __name__ == '__main__':

    # getcurrentCartesianCommand(prefix)

    rospy.init_node('service_server')
    # with open('z.csv', 'w') as f:
    service = rospy.Service('robot_calibration', trans_count, save_trans)
    # service = rospy.Service('word_count', WordCount, count_words)
    rospy.spin()
    print(poses)
    with open('poses/poses.txt',  'w') as tf:
        for p in poses:
            print(p)
            str_pose = ' '.join([str(i) for i in p])
            print(str_pose)
            tf.write(str_pose+'\n')
    print('over!')
