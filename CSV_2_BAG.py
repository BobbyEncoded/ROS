+#!/usr/bin/env python
+# -*- coding: utf-8 -*-
+
+# Software License Agreement (BSD License)
+#
+# Copyright (c) 2016, Martin Guenther
+# All rights reserved.
+#
+# Redistribution and use in source and binary forms, with or without
+# modification, are permitted provided that the following conditions
+# are met:
+#
+#  * Redistributions of source code must retain the above copyright
+#    notice, this list of conditions and the following disclaimer.
+#  * Redistributions in binary form must reproduce the above
+#    copyright notice, this list of conditions and the following
+#    disclaimer in the documentation and/or other materials provided
+#    with the distribution.
+#  * Neither the name of Willow Garage, Inc. nor the names of its
+#    contributors may be used to endorse or promote products derived
+#    from this software without specific prior written permission.
+#
+# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
+# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
+# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
+# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
+# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
+# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
+# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
+# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
+# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
+# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
+# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
+# POSSIBILITY OF SUCH DAMAGE.
+
+'''This is a converter for the Rawseeds Datasets to ROSbag files'''
+
+import rospy
+import rosbag
+from sensor_msgs.msg import LaserScan
+from math import pi
+from tf2_msgs.msg import TFMessage
+from geometry_msgs.msg import TransformStamped
+import tf
+
+def make_tf_msg(x, y, theta, t): #This sets up the transform maker, how the ROSbag file makes the TF topic
+    trans = TransformStamped()
+    trans.header.stamp = t
+    trans.header.frame_id = '/odom'
+    trans.child_frame_id = '/laser'
+    trans.transform.translation.x = x
+    trans.transform.translation.y = y
+    q = tf.transformations.quaternion_from_euler(0, 0, theta)
+    trans.transform.rotation.x = q[0]
+    trans.transform.rotation.y = q[1]
+    trans.transform.rotation.z = q[2]
+    trans.transform.rotation.w = q[3]
+
+    msg = TFMessage()
+    msg.transforms.append(trans)
+    return msg
+
+with rosbag.Bag('rawseeds.bag', 'w') as bag: #Create the rawseeds.bag bag file and use it
+    with open('Bicocca_2009-02-25b-SICK_FRONT.csv') as dataset: #Open the Sick_Front.csv and use it
+        for line in dataset.readlines(): #For each line in the dataset, which is the CSV file
+            line = line.strip() #Get the line
+            tokens = line.split(', ') #And break it into an array of each CSV part
+            if len(tokens) <= 2: #Ignore the terms if they are less than 2
+                continue
+            if 1:  #Ignore this line, we're not doing the .clf stuff
+                msg = LaserScan() #Sick_Front is a Laser Scan using the Sick sensor
+                num_scans = int(tokens[1]) #The number of scans is the first term
+
+                '''if num_scans != 181 or len(tokens) < num_scans + 9:
+                    rospy.logwarn("unsupported scan format")
+                    continue''' #This part is a check to make sure you're using the right file
+
+                msg.header.frame_id = 'SICK_FRONT'  #The message header tells that this is a laser scan
+                t = rospy.Time(float(tokens[0]))  #The first term states the time in seconds
+                msg.header.stamp = t  #And now it's the header
+                msg.angle_min = -90.0 / 180.0 * pi  #This is the minimum angle of the sensor scan
+                msg.angle_max = 90.0 / 180.0 * pi  #This is the maximum angle of the sensor scan
+                msg.angle_increment = pi / num_scans #Each increment is how far the sensor moves in angular movement between scans
+                msg.time_increment = 0.2 / 360.0  #This is how long each scan takes per angle?
+                msg.scan_time = 0.2  #This is how long each scan takes?
+                msg.range_min = 0.001  #This is the minimum range of the sensor?
+                msg.range_max = 50.0  #This is the maximum range of the sensor?
+                msg.ranges = [float(r) for r in tokens[2:(num_scans + 2)]] #This is the part where it pastes the data into that message of the bag file
+
+                bag.write('SICK_FRONT', msg, t)  #Create this and call it the "laser" topic in the bag file
+
+    with open('Bicocca_2009-02-25b-SICK_REAR.csv') as dataset: #Open the Sick_Front.csv and use it
+        for line in dataset.readlines(): #For each line in the dataset, which is the CSV file
+            line = line.strip() #Get the line
+            tokens = line.split(', ') #And break it into an array of each CSV part
+            if len(tokens) <= 2: #Ignore the terms if they are less than 2
+                continue
+            if 1:  #Ignore this line, we're not doing the .clf stuff
+                msg = LaserScan() #Sick_Front is a Laser Scan using the Sick sensor
+                num_scans = int(tokens[1]) #The number of scans is the first term
+
+                '''if num_scans != 181 or len(tokens) < num_scans + 9:
+                    rospy.logwarn("unsupported scan format")
+                    continue''' #This part is a check to make sure you're using the right file
+
+                msg.header.frame_id = 'SICK_REAR'  #The message header tells that this is a laser scan
+                t = rospy.Time(float(tokens[0]))  #The first term states the time in seconds
+                msg.header.stamp = t  #And now it's the header
+                msg.angle_min = -90.0 / 180.0 * pi  #This is the minimum angle of the sensor scan
+                msg.angle_max = 90.0 / 180.0 * pi  #This is the maximum angle of the sensor scan
+                msg.angle_increment = pi / num_scans #Each increment is how far the sensor moves in angular movement between scans
+                msg.time_increment = 0.2 / 360.0  #This is how long each scan takes per angle?
+                msg.scan_time = 0.2  #This is how long each scan takes?
+                msg.range_min = 0.001  #This is the minimum range of the sensor?
+                msg.range_max = 50.0  #This is the maximum range of the sensor?
+                msg.ranges = [float(r) for r in tokens[2:(num_scans + 2)]] #This is the part where it pastes the data into that message of the bag file
+
+                bag.write('SICK_REAR', msg, t)  #Create this and call it the "laser" topic in the bag file
+
+                #odom_x, odom_y, odom_theta = [float(r) for r in tokens[(num_scans + 2):(num_scans + 5)]]  #The .clf file writes the odometry data in the last 3 spots of the file, so this is irrelevant since we're using a CSV
+                #tf_msg = make_tf_msg(odom_x, odom_y, odom_theta, t) #This needs to be changed to real odometry data
+                #bag.write('tf', tf_msg, t)  #This writes the transform based on the odometry data
+
+            '''elif tokens[0] == 'ODOM':
+                odom_x, odom_y, odom_theta = [float(t) for t in tokens[1:4]]
+                t = rospy.Time(float(tokens[7]))
+                tf_msg = make_tf_msg(odom_x, odom_y, odom_theta, t)
+                bag.write('tf', tf_msg, t)'''
