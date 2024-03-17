#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge
command_pub = rospy.Publisher("/drone_command", edrone_msgs, queue_size=1)
found_object = True




class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [47.37,8.55,10] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [28.8,19.68,11.88]
		self.Ki = [0.064,0.08,0.072]
		self.Kd = [281.7,240,268.5]
		self.error = [0,0,0]
		self.Kp_error = [0,0,0]
		self.prev_values = [0,0,0]
		self.diff_values = [0,0,0]
		self.sum_item = [0,0,0]
		self.min_values = [1000,1000,1000]
		self.max_values = [2000,2000,2000]
		self.sample_time = 0.060
	   
		#-----------------------Add other required variables for pid here ----------------------------------------------








		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.060 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.command_pub1 = rospy.Publisher('/alt_error', Float64, queue_size=10)
		self.command_pub2 = rospy.Publisher('/pitch_error', Float64, queue_size=10)

		self.command_pub3 = rospy.Publisher('/roll_error',Float64, queue_size=10)

		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=10)




		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune, self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune, self.roll_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------






		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y

		self.drone_position[2] = msg.poses[0].position.z
	#print(self.drone_position[0])



		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self,alt):

		self.Kp[1] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly

		self.Ki[1] =alt.Ki * 0.008

		self.Kd[1] = alt.Kd * 0.3


	def roll_set_pid(self,alt):

		self.Kp[0] = alt.Kp * 0.06 ##This is just for an example. You can change the ratio/fraction value accordingly

		self.Ki[0] = alt.Ki * 0.008

		self.Kd[0] = alt.Kd * 0.3
















	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
		self.error[0] =  self.drone_position[0] - self.setpoint[0] #Kp for x

		self.error[1] =  self.drone_position[1] - self.setpoint[1]

		self.error[2] =  self.drone_position[2] - self.setpoint[2]
		print(self.error)

		self.diff_values[0] = self.error[0] - self.prev_values[0]

		self.diff_values[1] = self.error[1] - self.prev_values[1]
		self.diff_values[2] = self.error[2] - self.prev_values[2]
		for i in range(3):

			self.Kp_error[i] = self.Kp[i]*self.diff_values[i]
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.

	#	3. Callate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
		self.out_roll = float(self.error[0])*self.Kp[0] + float(self.sum_item[0])*self.Ki[0] + float(self.diff_values[0])*self.Kd[0]

		self.out_pitch = float(self.error[1]) + float(self.sum_item[1]) + float(self.prev_values[1])

		self.out_pitch = float(self.error[1])*self.Kp[1] + float(self.sum_item[1]*self.Ki[1]) + float(self.diff_values[1])*self.Kd[1]
		self.out_alt = float(self.error[2]) + float(self.sum_item[2]) + float(self.prev_values[2])

		self.out_alt = float(self.error[2])*self.Kp[2] + float(self.sum_item[2]*self.Ki[2]) + float(self.diff_values[2])*self.Kd[2]
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
		self.cmd.rcRoll = int(1500 - (self.out_roll))

		self.cmd.rcPitch = int(1500 + (self.out_pitch))

		self.cmd.rcThrottle = int(1500 + (self.out_alt))

		if self.cmd.rcPitch > self.max_values[1]:self.cmd.rcPitch = self.max_values[1]

		if self.cmd.rcRoll > self.max_values[0]:self.cmd.rcRoll = self.max_values[0]

		if self.cmd.rcThrottle > self.max_values[2]:self.cmd.rcThrottle = self.max_values[2]
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
		self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
		self.prev_values[0] = self.error[0]

		self.prev_values[1] = self.error[1]

		self.prev_values[2] = self.error[2]

		self.sum_item = self.sum_item + self.error

	#	8. Add error_sum
		self.command_pub1.publish(self.out_alt)

		self.command_pub2.publish(self.out_pitch)

		self.command_pub3.publish(self.out_roll)

		print(self.Kp[2],self.Ki[2],self.Kd[2])

		print(self.diff_values[0],self.diff_values[1],self.diff_values[2])









	#------------------------------------------------------------------------------------------------------------------------


		
		self.command_pub.publish(self.cmd)


def callback(data):
        global found_object

        # Used to convert between ROS and OpenCV images
        br = CvBridge()
        #coor = objectcenter()

        # Output debugging information to the terminal
        rospy.loginfo("receiving video frame")

        # Convert ROS Image message to OpenCV image
        img = br.imgmsg_to_cv2(data)
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        print(hsv.shape)
        # Display image
        cv2.imshow("camera", hsv)
        lower_bound = np.array([0, 0, 255],dtype="uint8")
        upper_bound = np.array([179,255, 255],dtype="uint8")
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        cv2.imshow("after_mask",mask)
        kernel = np.ones((7,7),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        segmented_img = cv2.bitwise_and(img, img, mask=mask)
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        output = cv2.drawContours(segmented_img, contours, -1, (0, 0, 255), 3)
        output = cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
        cv2.imshow("Output", output)
        print(contours)
        cx, cy = (None, None)
        dist_x_sum = 0
        dist_y_sum = 0
        for i in contours:
            M = cv2.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.drawContours(img, [i], -1, (0, 255, 0), 2)
                cv2.circle(img, (cx, cy), 7, (0, 0, 255), -1)
                cv2.putText(img, "center", (cx - 20, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                print(cx,cy)
            #coor.dist_x = 480 - cx
            #coor.dist_y = 640 - cy
            #obj_location = rospy.Publisher('/object_location', object_center, queue_size=1)
        
        if (cx is not None) and (cy is not None):
        
            cmd = edrone_msgs()
            
            dist_x = -(240-cx)
            dist_y = -(320-cy)
            
            dist_x_prev = dist_x
            dist_x_sum += dist_x
            
            dist_y_prev = dist_y
            dist_y_sum += dist_y
            found_object = True
        else:
            found_object = False
            #self.cmd.rcRoll = int(1500+(self.roll_error*self.Kp[0] + (self.roll_error-self.roll_prev_error)*self.Kd[0] + self.roll_sum_error*self.Ki[0]))

            cmd.rcPitch = int((dist_x)+(dist_x-dist_x_prev) + (dist_x_sum))
            cmd.rcRoll = int((dist_y)+(dist_y-dist_y_prev) + (dist_y_sum))
            command_pub.publish(cmd)

        cv2.waitKey(1)

def receive_message():

    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node('video_sub_py', anonymous=True)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/edrone/camera_rgb/image_raw', Image, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()

if __name__ == '__main__':
        while True:
            time_now = time.time()
            receive_message()
            while not found_object:
                spiral_cmd = edrone_msgs()
                spiral_cmd.rcPitch = 2 # TODO tune this
                spiral_cmd.rcYaw = 200*np.exp(-0.02*(time.time()-time_now)) # TODO tune this decay
                time_now = time.time()
                time.sleep(200)
