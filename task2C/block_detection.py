#The aim of this task is to scan the entire city area(arena) using drone, detect object from the drone camera.

import numpy as np
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from edrone_client.msg import *
import time
from sentinel_solution.msg import objectcenter


command_pub = rospy.Publisher("/drone_command", edrone_msgs, queue_size=1)
found_object = True


def callback(data):
    global found_object

    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    coor = objectcenter()

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
        coor.dist_x = 480 - cx
        coor.dist_y = 640 - cy
        obj_location = rospy.Publisher('/object_location', object_center, queue_size=1)

    if (cx is not None) and (cy is not None):
        P = 0.1# TODO tune this to get proper shit
        I = 0
        D = 10
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
        self.cmd.rcRoll = int(1500+(self.roll_error*self.Kp[0] + (self.roll_error-self.roll_prev_error)*self.Kd[0] + self.roll_sum_error*self.Ki[0]))

        cmd.rcPitch = int(P*(dist_x)+(dist_x-dist_x_prev)*D + (dist_x_sum)*I)
        cmd.rcRoll = int(P*(dist_y)+(dist_y-dist_y_prev)*D + (dist_y_sum)*I)
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
