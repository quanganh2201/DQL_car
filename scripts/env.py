import rclpy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.msg import ModelState
from rclpy.time import Time
from rclpy.node import Node
from datetime import datetime
from std_srvs.srv import Trigger
import time

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

import sys

MIN_DISTANCE=0.5

class Env(Node):
    def __init__(self):
        self.delclient = self.create_client(DeleteEntity, '/delete_entity')
        self.delresult = False

        self.spawnclient = self.create_client(SpawnEntity, '/spawn_entity')
        self.req = SpawnEntity.Request()
        self.setPosPub = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)
        self.velPub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.callShutdown = False

        self.depth_subcription = self.create_subscription(Image,'/camera_link/depth/image_raw',self.process_data_depth,10)
        self.subscription = self.create_subscription(
            Image,
            '/camera_link/image_raw',
            self.image_callback,
            10)
        self.model = YOLO('~/yolobot/src/yolobot_recognition/scripts/yolov8n.pt')
        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        self.view_depth = None
        self.depth_img = None
        self.posX = np.empty([1],type = int)
        self.posY = np.empty([1],type = int)
        self.range = np.empty([1],type = float)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # FOR SPAWNING MODEL IN GAZEBO
    def delete_entity(self, name):
        request = DeleteEntity.Request()
        request.name = name

        # Call the service
        future = self.delclient.call_async(request)
        future.add_done_callback(self.delete_entity_callback)

    def delete_entity_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.delresult = True
            else:
                self.delresult = False
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def spawn_entity_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info(
                    'Spawn entity response: success={}, status_message={}'.format(
                        response.success, response.status_message
                    )
                )
            else:
                self.get_logger().error('Service call failed')
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % e)

    def call_spawn_entity_service(self, name, xml_file_path, x, y, z):
        try:
            with open(xml_file_path, 'r') as file:
                xml_content = file.read()

            self.req.name = name
            self.req.xml = xml_content
            self.req.initial_pose.position.x = x
            self.req.initial_pose.position.y = y
            self.req.initial_pose.position.z = z

            future = self.spawnclient.call_async(self.req)
            future.add_done_callback(self.spawn_entity_callback)
        except Exception as e:
            self.get_logger().error('Failed to call spawn_entity service: %r' % e)

    #HANDLE CAMERA STREAM
    def process_data_depth(self, data):
        self.view_depth = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        self.view_depth = np.array(self.view_depth, dtype=np.float32)
        #alpha: contrast 0 -127, beta: brightness 0 -100
        self.depth_img = cv2.convertScaleAbs(self.view_depth, alpha=10 , beta=30)
        cv2.imshow('view', self.depth_img)
        #cv2.imshow('view0', self.view_depth)

    def image_callback(self, msg):
        self.posX = np.empty([1],type = int)
        self.posY = np.empty([1],type = int)
        self.range = np.empty([1],type = float)

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        results = self.model(cv_image)
        print(cv_image.size)
        height, width, _ = cv_image.shape
        img_center_x = cv_image.shape[0] // 2
        img_center_y = cv_image.shape[1] // 2
        
        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                top = int(b[0])
                left = int(b[1])
                bottom = int(b[2])
                right = int(b[3])

                center_y = (left + right) // 2
                center_x = (top + bottom) // 2
                depth = 10

                # Draw the center point
                cv2.circle(cv_image, (center_x, center_y), radius=5, color=(255, 255, 0), thickness=-1)
                if self.view_depth is None:
                    pass
                else:
                    depth = self.view_depth[center_y, center_x]
                    cv2.putText(cv_image, f"{depth :.2f}m", (center_x + 5, center_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    np.append(self.posX, center_x)
                    np.append(self.posY, center_y)
                    np.append(self.range, depth)

        annotated_frame = results[0].plot(labels = True)
        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame)  
        self.img_pub.publish(img_msg)
        

    def getState(self):
        min_ran = np.min(self.range)
        posi_x=self.posX(np.argmin(self.range))
        posi_y= self.posY(np.argmin(self.range))
        return posi_x, posi_y, min_ran

    def setReward(self, state, done, action):
        pre_action = action[-2]
        now_action = action[-1]
        min_ran = state[-1]
        if action == 0:#straight
            r_action = +0.2
        else:
            r_action = -0.1
        if (pre_action == 1 and now_action == 2) or (pre_action == 2 and now_action == 1):
            r_change = -0.3
        else:
            r_change = +0.2
        if min_ran < MIN_DISTANCE:
            r_dis = -0.5
        else:
            r_dis = +0.05
        reward = r_change + r_action + r_dis
        return reward

    def step(self, action):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        self.velPub.publish(vel_cmd)

        state, done = self.getState()
        reward = self.setReward(state, done, action)

        return np.asarray(state), reward, done

    def reset(self):
        self.delete_entity('burger')

        #SHOULD HAVE A TIMER HERE

        self.call_spawn_entity_service('burger', XML_FILE_PATH, X_INIT, Y_INIT, THETA_INIT)
 
        state, done = self.getState()

        return np.asarray(state)
    
    def timer_callback(self):
        pass