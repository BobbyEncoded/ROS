#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Martin Guenther
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

'''This is a converter for the Rawseeds Datasets to ROSbag files'''

import rospy
import rosbag
import numpy #Used for matrices and quaternion transform for IMU, don't forget to sudo apt-get install python-numpy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from math import pi
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import tf

#The below snippet is taken from http://www.lfd.uci.edu/~gohlke/code/transformations.py.html
def quaternion_from_matrix(matrix, isprecise=False):
    """Return quaternion from rotation matrix.
    If isprecise is True, the input matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.
    >>> q = quaternion_from_matrix(numpy.identity(4), True)
    >>> numpy.allclose(q, [1, 0, 0, 0])
    True
    >>> q = quaternion_from_matrix(numpy.diag([1, -1, -1, 1]))
    >>> numpy.allclose(q, [0, 1, 0, 0]) or numpy.allclose(q, [0, -1, 0, 0])
    True
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R, True)
    >>> numpy.allclose(q, [0.9981095, 0.0164262, 0.0328524, 0.0492786])
    True
    >>> R = [[-0.545, 0.797, 0.260, 0], [0.733, 0.603, -0.313, 0],
    ...      [-0.407, 0.021, -0.913, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.19069, 0.43736, 0.87485, -0.083611])
    True
    >>> R = [[0.395, 0.362, 0.843, 0], [-0.626, 0.796, -0.056, 0],
    ...      [-0.677, -0.498, 0.529, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.82336615, -0.13610694, 0.46344705, -0.29792603])
    True
    >>> R = random_rotation_matrix()
    >>> q = quaternion_from_matrix(R)
    >>> is_same_transform(R, quaternion_matrix(q))
    True
    >>> R = euler_matrix(0.0, 0.0, numpy.pi/2.0)
    >>> numpy.allclose(quaternion_from_matrix(R, isprecise=False),
    ...                quaternion_from_matrix(R, isprecise=True))
    True
    """
    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
    if isprecise:
        q = numpy.empty((4, ))
        t = numpy.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 1, 2, 3
            if M[1, 1] > M[0, 0]:
                i, j, k = 2, 3, 1
            if M[2, 2] > M[i, i]:
                i, j, k = 3, 1, 2
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = numpy.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = numpy.linalg.eigh(K)
        q = V[[3, 0, 1, 2], numpy.argmax(w)]
    if q[0] < 0.0:
        numpy.negative(q, q)
    return q

def make_tf_msg(x, y, theta, t): #This sets up the transform maker, how the ROSbag file makes the TF topic
    trans = TransformStamped()
    trans.header.stamp = t
    trans.header.frame_id = '/odom'
    trans.child_frame_id = '/base_footprint'
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    q = tf.transformations.quaternion_from_euler(0, 0, theta)
    trans.transform.rotation.x = q[0]
    trans.transform.rotation.y = q[1]
    trans.transform.rotation.z = q[2]
    trans.transform.rotation.w = q[3]

    msg = TFMessage()
    msg.transforms.append(trans)
    return msg

def make_tf2_msg(t): #This sets up the transform maker, how the ROSbag file makes the TF topic
    trans = TransformStamped()
    trans.header.stamp = t
    trans.header.frame_id = '/base_link'
    trans.child_frame_id = '/SICK_FRONT'
    trans.transform.translation.x = .08
    trans.transform.translation.y = 0
    trans.transform.translation.z = .450
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    trans.transform.rotation.x = q[0]
    trans.transform.rotation.y = q[1]
    trans.transform.rotation.z = q[2]
    trans.transform.rotation.w = q[3]

    msg = TFMessage()
    msg.transforms.append(trans)
    return msg

def make_tf3_msg(t): #This sets up the transform maker, how the ROSbag file makes the TF topic
    trans = TransformStamped()
    trans.header.stamp = t
    trans.header.frame_id = '/base_footprint'
    trans.child_frame_id = '/base_link'
    trans.transform.translation.x = 0
    trans.transform.translation.y = 0
    trans.transform.translation.z = 0
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    trans.transform.rotation.x = q[0]
    trans.transform.rotation.y = q[1]
    trans.transform.rotation.z = q[2]
    trans.transform.rotation.w = q[3]

    msg = TFMessage()
    msg.transforms.append(trans)
    return msg

def make_tf4_msg(t): #This sets up the transform maker, how the ROSbag file makes the TF topic
    trans = TransformStamped()
    trans.header.stamp = t
    trans.header.frame_id = '/base_link'
    trans.child_frame_id = '/IMU'
    trans.transform.translation.x = -0.192
    trans.transform.translation.y = -0.007
    trans.transform.translation.z = 0.537
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    trans.transform.rotation.x = q[0]
    trans.transform.rotation.y = q[1]
    trans.transform.rotation.z = q[2]
    trans.transform.rotation.w = q[3]

def make_tf5_msg(t): #This sets up the transform maker, how the ROSbag file makes the TF topic
    trans = TransformStamped()
    trans.header.stamp = t
    trans.header.frame_id = '/base_link'
    trans.child_frame_id = '/SICK_REAR'
    trans.transform.translation.x = -0.463
    trans.transform.translation.y = 0.001
    trans.transform.translation.z = 0.454
    q = tf.transformations.quaternion_from_euler(0, 0, pi)
    trans.transform.rotation.x = q[0]
    trans.transform.rotation.y = q[1]
    trans.transform.rotation.z = q[2]
    trans.transform.rotation.w = q[3]

    msg = TFMessage()
    msg.transforms.append(trans)
    return msg

with rosbag.Bag('rawseeds.bag', 'w') as bag: #Create the rawseeds.bag bag file and use it
    with open('SICK_FRONT_Matched_CSV.csv') as dataset: #Open the Sick_Front.csv and use it
        for line in dataset.readlines(): #For each line in the dataset, which is the CSV file
            line = line.strip() #Get the line
            tokens = line.split(',') #And break it into an array of each CSV part
            if len(tokens) <= 2: #Ignore the terms if they are less than 2
                continue
            if 1:  #Ignore this line, we're not doing the .clf stuff
                msg = LaserScan() #Sick_Front is a Laser Scan using the Sick sensor
                num_scans = int(tokens[1]) #The number of scans is the first term
                '''if num_scans != 181 or len(tokens) < num_scans + 9:
                    rospy.logwarn("unsupported scan format")
                    continue''' #This part is a check to make sure you're using the right file

                msg.header.frame_id = 'SICK_FRONT'  #The message header tells that this is a laser scan
                t = rospy.Time(float(tokens[0]))  #The first term states the time in seconds
                msg.header.stamp = t  #And now it's the header
                msg.angle_min = -90.0 / 180.0 * pi  #This is the minimum angle of the sensor scan
                msg.angle_max = 90.0 / 180.0 * pi  #This is the maximum angle of the sensor scan
                msg.angle_increment = pi / num_scans #Each increment is how far the sensor moves in angular movement between scans
                msg.time_increment = 0.2 / 360.0  #This is how long each scan takes per angle?
                msg.scan_time = 0.2  #This is how long each scan takes?
                msg.range_min = 0.001  #This is the minimum range of the sensor?
                msg.range_max = 50.0  #This is the maximum range of the sensor?
                msg.ranges = [float(r) for r in tokens[2:(num_scans + 2)]] #This is the part where it pastes the data into that message of the bag file
		msg.intensities = []

                bag.write('SICK_FRONT', msg, t)  #Create this and call it the "SICK_FRONT" topic in the bag file

    with open('SICK_REAR_Matched_CSV.csv') as dataset: #Open the Sick_Front.csv and use it
        for line in dataset.readlines(): #For each line in the dataset, which is the CSV file
            line = line.strip() #Get the line
            tokens = line.split(',') #And break it into an array of each CSV part
            if len(tokens) <= 2: #Ignore the terms if they are less than 2
                continue
            if 1:  #Ignore this line, we're not doing the .clf stuff
                msg = LaserScan() #Sick_Front is a Laser Scan using the Sick sensor
                num_scans = int(tokens[1]) #The number of scans is the first term

                '''if num_scans != 181 or len(tokens) < num_scans + 9:
                    rospy.logwarn("unsupported scan format")
                    continue''' #This part is a check to make sure you're using the right file

                msg.header.frame_id = 'SICK_REAR'  #The message header tells that this is a laser scan
                t = rospy.Time(float(tokens[0]))  #The first term states the time in seconds
                msg.header.stamp = t  #And now it's the header
                msg.angle_min = -90.0 / 180.0 * pi  #This is the minimum angle of the sensor scan
                msg.angle_max = 90.0 / 180.0 * pi  #This is the maximum angle of the sensor scan
                msg.angle_increment = pi / num_scans #Each increment is how far the sensor moves in angular movement between scans
                msg.time_increment = 0.2 / 360.0  #This is how long each scan takes per angle?
                msg.scan_time = 0.2  #This is how long each scan takes?
                msg.range_min = 0.001  #This is the minimum range of the sensor?
                msg.range_max = 50.0  #This is the maximum range of the sensor?
                msg.ranges = [float(r) for r in tokens[2:(num_scans + 2)]] #This is the part where it pastes the data into that message of the bag file
		msg.intensities = []


                bag.write('SICK_REAR', msg, t)  #Create this and call it the "SICK_REAR" topic in the bag file

    with open('IMU_Matched_CSV.csv') as dataset: #Open the IMU file and use it
        for line in dataset.readlines(): #For each line in the dataset, which is the CSV file
            line = line.strip() #Get the line
            tokens = line.split(',') #And break it into an array of each CSV part
            if len(tokens) <= 2: #Ignore the terms if they are less than 2
                continue
            msg = Imu() #IMU_STRETCHED is the IMU datatype using the IMU_STRETCHED sensor

            msg.header.frame_id = 'IMU'  #The message header labels the topic
            t = rospy.Time(float(tokens[0]))  #The first term states the time in seconds
            msg.header.stamp = t  #And now it's the header
            imumatrixelements = numpy.array([[float(tokens[11]), float(tokens[12]), float(tokens[13])], [float(tokens[14]), float(tokens[15]), float(tokens[16])], [float(tokens[17]), float(tokens[18]), float(tokens[19])]])
            imuquaternion = quaternion_from_matrix(imumatrixelements) #This returns the quaternion from the matrix we just created
            #print(imuquaternion)
            msg.orientation.x = float(imuquaternion[1])
            msg.orientation.y = float(imuquaternion[2])
            msg.orientation.z = float(imuquaternion[3])
            msg.orientation.w = float(imuquaternion[0])

            '''We are now going to define the angular velocities and linear velocities in the IMU type in the bag file by their appropriate numbers from the rawseeds files'''

            angvel = Vector3()  #IMU's angular_velocity variable requires that the x, y, and z coordinates are given using Vector3() library.  This sets that up.  Therefore, Vector3 must be imported from geometry_msgs for this to work.
            angvel.x = float(tokens[5])
            angvel.y = float(tokens[6])
            angvel.z = float(tokens[7])
            msg.angular_velocity.x = angvel.x
            msg.angular_velocity.y = angvel.y
            msg.angular_velocity.z = angvel.z

            linacc = Vector3()
            linacc.x = float(tokens[2])
            linacc.y = float(tokens[3])
            linacc.z = float(tokens[4])
            msg.linear_acceleration.x = linacc.x
            msg.linear_acceleration.y = linacc.y
            msg.linear_acceleration.z = linacc.z

	    bag.write('IMU', msg, t) #Create this and call it the "IMU" topic in the bag file

    with open('Bicocca_2009-02-25b-ODOMETRY_XYT_Matched.csv') as dataset: #Open the Sick_Front.csv and use it
	count = 0
	for line in dataset.readlines(): #For each line in the dataset, which is the CSV file
            line = line.strip() #Get the line
            tokens = line.split(',') #And break it into an array of each CSV part
	    count = count + 1
            if len(tokens) <= 2: #Ignore the terms if they are less than 2
                continue
	    if 1:  #Ignore this line, we're not doing the .clf stuff
		t = rospy.Time(float(tokens[0]))
                odom_x, odom_y, odom_theta = [float(r) for r in tokens[(4):(7)]]  #Collects the odometry data in the file and loads it 
                tf_msg = make_tf_msg(odom_x, odom_y, odom_theta, t) #This needs to be changed to real odometry data
                bag.write('tf', tf_msg, t)  #This writes the transform based on the odometry data
		tf_msg = make_tf2_msg(t)
                bag.write('tf', tf_msg, t)  #This writes the transform for the SICK_FRONT to the base_link
		tf_msg = make_tf3_msg(t)
                bag.write('tf', tf_msg, t)  #This writes the transform for the footprint based on the base link
		'''tf_msg = make_tf4_msg(t)
                bag.write('tf', tf_msg, t)  #This writes the transform for the IMU based on the base link'''
	        tf_msg = make_tf5_msg(t)
                bag.write('tf', tf_msg, t)  #This writes the transform for the SICK_REAR based on the base link



            '''elif tokens[0] == 'ODOM':
                odom_x, odom_y, odom_theta = [float(t) for t in tokens[1:4]]
                t = rospy.Time(float(tokens[7]))
                tf_msg = make_tf_msg(odom_x, odom_y, odom_theta, t)
                bag.write('tf', tf_msg, t)'''
