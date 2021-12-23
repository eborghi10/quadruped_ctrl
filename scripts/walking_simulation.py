#!/usr/bin/env python

import concurrent.futures
import ctypes
import cv2
import math
import numpy as np
import os
import pybullet as p
import pybullet_data
import random
import rospkg
import rospy
import tf2_ros
import threading

from pybullet_utils import gazebo_world_parser

from cv_bridge import CvBridge
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from quadruped_ctrl.srv import QuadrupedCmd, QuadrupedCmdResponse
from sensor_msgs.msg import Image, Imu, JointState, PointCloud2, PointField
from tf_conversions import transformations
from whole_body_state_msgs.msg import WholeBodyState
from whole_body_state_msgs.msg import JointState as WBJointState
from whole_body_state_msgs.msg import ContactState as WBContactState


class StructPointer(ctypes.Structure):
    _fields_ = [("eff", ctypes.c_double * 12)]


class WalkingSimulation(object):
    def __init__(self):
        self.terrain = "racetrack"
        self.camera = True
        self.get_last_vel = [0] * 3
        self.robot_height = 0.30
        self.motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
        self.init_new_pos = [0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.__init_ros()
        self.__load_controller()
        self.__init_simulator()

        add_thread = threading.Thread(target=self.__thread_job)
        add_thread.start()

        if self.camera:
            add_thread_1 = threading.Thread(target=self.__camera_update)
            add_thread_1.start()

    def __init_ros(self):
        self.terrain = rospy.get_param('/simulation/terrain')
        self.camera = rospy.get_param('/simulation/camera')
        self.lateralFriction = rospy.get_param('/simulation/lateralFriction')
        self.spinningFriction = rospy.get_param('/simulation/spinningFriction')
        self.freq = rospy.get_param('/simulation/freq')
        self.stand_kp = rospy.get_param('/simulation/stand_kp')
        self.stand_kd = rospy.get_param('/simulation/stand_kd')
        self.joint_kp = rospy.get_param('/simulation/joint_kp')
        self.joint_kd = rospy.get_param('/simulation/joint_kd')
        rospy.loginfo("lateralFriction = " + str(self.lateralFriction) +
                      " spinningFriction = " + str(self.spinningFriction))
        rospy.loginfo(" freq = " + str(self.freq) + " PID = " +
                      str([self.stand_kp, self.stand_kd, self.joint_kp, self.joint_kd]))

        self.s0 = rospy.Service('gait_type', QuadrupedCmd, self.__callback_gait)
        self.s1 = rospy.Service('robot_mode', QuadrupedCmd, self.__callback_mode)
        self.s2 = rospy.Subscriber("cmd_vel", Twist, self.__callback_body_vel, buff_size=30)
        self.s3 = rospy.Subscriber("elevation_mapping/elevation_map", GridMap, self.__callback_elevation_map)

        self.robot_tf = tf2_ros.TransformBroadcaster()

    def __load_controller(self):
        self.path = rospkg.RosPack().get_path('quadruped_ctrl')
        so_file = self.path.replace('src/quadruped_ctrl', 'devel/lib/libquadruped_ctrl.so')
        if(not os.path.exists(so_file)):
            so_file = self.path.replace('src/quadruped_ctrl', 'build/lib/libquadruped_ctrl.so')
        if(not os.path.exists(so_file)):
            rospy.logerr("cannot find cpp.so file")
        self.cpp_gait_ctrller = ctypes.cdll.LoadLibrary(so_file)
        self.cpp_gait_ctrller.torque_calculator.restype = ctypes.POINTER(StructPointer)
        rospy.loginfo("find so file = " + so_file)

    def __init_simulator(self):
        robot_start_pos = [0, 0, self.robot_height]
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.resetSimulation()
        p.setTimeStep(1.0/self.freq)
        p.setGravity(0, 0, -9.81)
        self.reset = p.addUserDebugParameter("reset", 1, 0, 0)
        self.low_energy_mode = p.addUserDebugParameter("low_energy_mode", 1, 0, 0)
        self.high_performance_mode = p.addUserDebugParameter("high_performance_mode", 1, 0, 0)
        p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])

        heightPerturbationRange = 0.06
        numHeightfieldRows = 256
        numHeightfieldColumns = 256
        if self.terrain == "plane":
            planeShape = p.createCollisionShape(shapeType=p.GEOM_PLANE)
            ground_id = p.createMultiBody(0, planeShape)
            p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
            p.changeDynamics(ground_id, -1, lateralFriction=self.lateralFriction)
        elif self.terrain == "random1":
            heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns
            for j in range(int(numHeightfieldColumns/2)):
                for i in range(int(numHeightfieldRows/2)):
                    height = random.uniform(0, heightPerturbationRange)
                    heightfieldData[2*i+2*j*numHeightfieldRows] = height
                    heightfieldData[2*i+1+2*j*numHeightfieldRows] = height
                    heightfieldData[2*i+(2*j+1)*numHeightfieldRows] = height
                    heightfieldData[2*i+1+(2*j+1)*numHeightfieldRows] = height
            terrainShape = p.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                meshScale=[.05, .05, 1],
                heightfieldTextureScaling=(numHeightfieldRows-1)/2,
                heightfieldData=heightfieldData,
                numHeightfieldRows=numHeightfieldRows,
                numHeightfieldColumns=numHeightfieldColumns)
            ground_id = p.createMultiBody(0, terrainShape)
            p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
            p.changeDynamics(ground_id, -1, lateralFriction=self.lateralFriction)
        elif self.terrain == "random2":
            terrain_shape = p.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                meshScale=[.5, .5, .5],
                fileName="heightmaps/ground0.txt",
                heightfieldTextureScaling=128)
            ground_id = p.createMultiBody(0, terrain_shape)
            textureId = p.loadTexture(self.path + "/models/grass.png")
            p.changeVisualShape(ground_id, -1, textureUniqueId=textureId)
            p.resetBasePositionAndOrientation(ground_id, [1, 0, 0.2], [0, 0, 0, 1])
            p.changeDynamics(ground_id, -1, lateralFriction=self.lateralFriction)
        elif self.terrain == "stairs":
            planeShape = p.createCollisionShape(shapeType=p.GEOM_PLANE)
            ground_id = p.createMultiBody(0, planeShape)
            p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
            # Stair as a combination of boxes
            sh_colBox = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2.5, 2.5, 0.2])
            for step in range(4):
                p.createMultiBody(baseMass=0, baseCollisionShapeIndex=sh_colBox,
                                  basePosition=[3.75 + 0.33 * step, 0, -0.2 + (step+1) * 0.25],
                                  baseOrientation=[0, 0, 0, 1])
        elif self.terrain == "racetrack":
            os.chdir(self.path)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
            gazebo_world_parser.parseWorld(p, filepath="worlds/racetrack_day.world")
            p.configureDebugVisualizer(shadowMapResolution=8192)
            p.configureDebugVisualizer(shadowMapWorldSize=25)
            # Enable rendering after loading the world
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        # Disable visualization of cameras in pybullet GUI
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

        # Enable this if you want better performance
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

        # TODO: Get the URDF from robot_description parameter (or URDF file in the repo)
        self.boxId = p.loadURDF("mini_cheetah/mini_cheetah.urdf", robot_start_pos, useFixedBase=False)
        p.changeDynamics(self.boxId, 3, spinningFriction=self.spinningFriction)
        p.changeDynamics(self.boxId, 7, spinningFriction=self.spinningFriction)
        p.changeDynamics(self.boxId, 11, spinningFriction=self.spinningFriction)
        p.changeDynamics(self.boxId, 15, spinningFriction=self.spinningFriction)

        self.__reset_robot()

    def __reset_robot(self):
        if self.terrain == "racetrack":
            robot_z = 0.4
        else:
            robot_z = self.robot_height
        p.resetBasePositionAndOrientation(
            self.boxId, [0, 0, robot_z], [0, 0, 0, 1])
        p.resetBaseVelocity(self.boxId, [0, 0, 0], [0, 0, 0])
        for j in range(12):
            p.resetJointState(
                self.boxId, self.motor_id_list[j], self.init_new_pos[j], self.init_new_pos[j+12])
        self.cpp_gait_ctrller.init_controller(
            self.__convert_type(self.freq),
            self.__convert_type([self.stand_kp, self.stand_kd, self.joint_kp, self.joint_kd]))

        for _ in range(10):
            p.stepSimulation()
            imu_data, leg_data, _, _ = self.__get_data_from_sim()
            self.cpp_gait_ctrller.pre_work(self.__convert_type(
                imu_data), self.__convert_type(leg_data["state"]))

        p.setJointMotorControlArray(bodyUniqueId=self.boxId,
                                    jointIndices=self.motor_id_list,
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0]*len(self.motor_id_list))

        self.cpp_gait_ctrller.set_robot_mode(self.__convert_type(1))

    def run(self):
        rate = rospy.Rate(self.freq)  # Hz
        reset_flag = p.readUserDebugParameter(self.reset)
        low_energy_flag = p.readUserDebugParameter(self.low_energy_mode)
        high_performance_flag = p.readUserDebugParameter(self.high_performance_mode)
        while not rospy.is_shutdown():
            # check reset button state
            if(reset_flag < p.readUserDebugParameter(self.reset)):
                reset_flag = p.readUserDebugParameter(self.reset)
                rospy.logwarn("reset the robot")
                self.__reset_robot()
            if(low_energy_flag < p.readUserDebugParameter(self.low_energy_mode)):
                low_energy_flag = p.readUserDebugParameter(self.low_energy_mode)
                rospy.loginfo("set robot to low energy mode")
                self.cpp_gait_ctrller.set_robot_mode(self.__convert_type(1))
            if(high_performance_flag < p.readUserDebugParameter(self.high_performance_mode)):
                high_performance_flag = p.readUserDebugParameter(self.high_performance_mode)
                rospy.loginfo("set robot to high performance mode")
                self.cpp_gait_ctrller.set_robot_mode(self.__convert_type(0))

            self.__simulation_step()

            rate.sleep()

    def __simulation_step(self):
        # get data from simulator
        imu_data, leg_data, base_pos, contact_points = self.__get_data_from_sim()

        # pub msg
        self.__pub_nav_msg(base_pos, imu_data)
        self.__pub_ground_truth_pose(base_pos, imu_data)
        self.__pub_imu_msg(imu_data)
        self.__pub_joint_states(leg_data)
        self.__pub_whole_body_state(imu_data, leg_data, base_pos, contact_points)

        # call cpp function to calculate mpc tau
        # tau = self.cpp_gait_ctrller.torque_calculator(self.__convert_type(
        #     imu_data), self.__convert_type(leg_data["state"]))

        # set tau to simulator
        # p.setJointMotorControlArray(bodyUniqueId=self.boxId,
        #                             jointIndices=self.motor_id_list,
        #                             controlMode=p.TORQUE_CONTROL,
        #                             forces=tau.contents.eff)

        # p.stepSimulation()

    def __get_ros_depth_image_msg(self, depth):
        depth_raw_image = self.far * self.near / (self.far - (self.far - self.near) * depth)
        depth_raw_image = (depth_raw_image * 1000).astype(np.uint16)
        msg = CvBridge().cv2_to_imgmsg(depth_raw_image)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "body"
        return msg

    def __get_ros_rgb_image_msg(self, rgba):
        image = cv2.cvtColor(np.uint8(rgba), code=cv2.COLOR_RGBA2RGB)
        msg = CvBridge().cv2_to_imgmsg(image)
        msg.header.stamp = rospy.Time().now()
        msg.header.frame_id = "cam"
        msg.encoding = "rgb8"
        return msg

    def calIntrinsicMatrix(self):
        f = math.sqrt(self.width * self.width / 4.0 + self.height * self.height / 4.0) / 2.0 / \
                math.tan(self.fov / 2.0 / 180.0 * math.pi)
        return (f, 0.0, self.width / 2.0 - 0.5, 0.0, f, self.height / 2.0 - 0.5, 0.0, 0.0, 1.0)

    def __generate_scene_pointcloud(self, depth, rgba):
        '''Generate point cloud from depth image and color image
        Args:
            depth(str / np.array): Depth image path or depth.
            rgb(str / np.array): RGB image path or RGB values.
            intrinsics(np.array): Camera intrinsics matrix.
            depth_scale(float): The depth factor.
        Returns:
            np.array(float), np.array(int): points and colors
        '''
        intrinsics = np.array(self.calIntrinsicMatrix()).reshape((3, 3))
        depth_scale = 1.0
        depths = self.far * self.near / (self.far - (self.far - self.near) * depth)
        colors = cv2.cvtColor(np.uint8(rgba), code=cv2.COLOR_RGBA2RGB)

        fx, fy = intrinsics[0, 0], intrinsics[1, 1]
        cx, cy = intrinsics[0, 2], intrinsics[1, 2]

        xmap, ymap = np.arange(colors.shape[1]), np.arange(colors.shape[0])
        xmap, ymap = np.meshgrid(xmap, ymap)

        points_z = depths / depth_scale
        points_x = (xmap - cx) / fx * points_z
        points_y = (ymap - cy) / fy * points_z

        mask = (points_z > 0)
        points = np.stack([points_x, points_y, points_z], axis=-1)
        points = points[mask]
        colors = colors[mask]
        return points, colors

    def __get_ros_pointcloud_msg(self, depth, rgba):
        points, colors = self.__generate_scene_pointcloud(depth, rgba)
        points = points.astype(np.float32)
        msg = PointCloud2()
        msg.header.stamp = rospy.Time().now()

        C = np.zeros((colors[:, 0].size, 4), dtype=np.uint8)

        C[:, 0] = colors[:, 2].astype(np.uint8)
        C[:, 1] = colors[:, 1].astype(np.uint8)
        C[:, 2] = colors[:, 0].astype(np.uint8)

        C = C.view("uint32")
        C = C.view("float32")
        pointsColor = np.zeros((points.shape[0], 1), \
        dtype={
            "names": ( "x", "y", "z", "rgba" ),
            "formats": ( "f4", "f4", "f4", "f4" )} )

        points = points.astype(np.float32)

        pointsColor["x"] = points[:, 0].reshape((-1, 1))
        pointsColor["y"] = points[:, 1].reshape((-1, 1))
        pointsColor["z"] = points[:, 2].reshape((-1, 1))
        pointsColor["rgba"] = C
        msg.header.frame_id = "cam"
        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)

        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = int(np.isfinite(points).all())
        msg.data = pointsColor.tobytes()
        return msg

    # https://github.com/OCRTOC/OCRTOC_software_package/blob/master/pybullet_simulator/scripts/pybullet_env.py
    def __camera_update(self):
        rate = rospy.Rate(20)

        # Projection matrix parameters
        self.near = 0.01
        self.far = 3.0
        self.fov = 60
        step_index = 4
        self.width = int(320 / step_index)
        self.height = int(240 / step_index)
        self.aspect = float(self.width) / float(self.height)

        # Init ROS publishers
        self.pointcloud_publisher = rospy.Publisher("/cam0/depth/points", PointCloud2, queue_size=1)
        self.image_publisher = rospy.Publisher("/cam0/image_raw", Image, queue_size=1)
        self.depth_publisher = rospy.Publisher("/cam0/image_depth", Image, queue_size=1)

        rospy.loginfo("Starting camera thread")

        T1 = np.mat([[0, -1.0/2.0, np.sqrt(3.0)/2.0, 0.25], [-1, 0, 0, 0],
                     [0, -np.sqrt(3.0)/2.0, -1.0/2.0, 0], [0, 0, 0, 1]])

        cameraEyePosition = [0.3, 0, 0.26436384367425125]
        cameraTargetPosition = [1.0, 0, 0]
        cameraUpVector = [0, 0, 1]

        while not rospy.is_shutdown():
            cubePos, cubeOrn = p.getBasePositionAndOrientation(self.boxId)
            get_matrix = p.getMatrixFromQuaternion(cubeOrn)

            T2 = np.mat([[get_matrix[0], get_matrix[1], get_matrix[2], cubePos[0]],
                         [get_matrix[3], get_matrix[4], get_matrix[5], cubePos[1]],
                         [get_matrix[6], get_matrix[7], get_matrix[8], cubePos[2]],
                         [0, 0, 0, 1]])

            T3 = np.array(T2*T1)

            cameraEyePosition = T3[0:3, 3].tolist()
            cameraTargetPosition = (np.mat(T3) * np.array([[0], [0], [1], [1]]))[0:3]

            # Get quaternion from numpy homogeneus matrix
            cameraQuat = transformations.quaternion_from_matrix(T3)

            self.robot_tf.sendTransform(self.__fill_tf_message("world", "body", cubePos, cubeOrn))
            self.robot_tf.sendTransform(
                self.__fill_tf_message("world", "cam", cameraEyePosition, cameraQuat))
            self.robot_tf.sendTransform(
                self.__fill_tf_message("world", "tar", cameraTargetPosition, cubeOrn))

            viewMatrix = p.computeViewMatrix(
                cameraEyePosition, cameraTargetPosition, cameraUpVector)
            projectionMatrix = p.computeProjectionMatrixFOV(
                self.fov, self.aspect, self.near, self.far)
            _, _, rgba, depth, _ = p.getCameraImage(
                    self.width,
                    self.height,
                    viewMatrix=viewMatrix,
                    projectionMatrix=projectionMatrix,
                    shadow=1,
                    lightDirection=[1, 1, 1],
                    renderer=p.ER_BULLET_HARDWARE_OPENGL,
                    flags=p.ER_NO_SEGMENTATION_MASK)

            with concurrent.futures.ThreadPoolExecutor() as executor:
                f1 = executor.submit(self.__get_ros_depth_image_msg, depth)
                f2 = executor.submit(self.__get_ros_rgb_image_msg, rgba)
                f3 = executor.submit(self.__get_ros_pointcloud_msg, depth, rgba)

                r1 = f1.result()
                r2 = f2.result()
                r3 = f3.result()

                if(self.depth_publisher.get_num_connections() > 0):
                    self.depth_publisher.publish(r1)
                if(self.image_publisher.get_num_connections() > 0):
                    self.image_publisher.publish(r2)
                if(self.pointcloud_publisher.get_num_connections() > 0):
                    self.pointcloud_publisher.publish(r3)

            rate.sleep()

    def __convert_type(self, input):
        ctypes_map = {
            int: ctypes.c_int,
            float: ctypes.c_double,
            str: ctypes.c_char_p,
                     }
        input_type = type(input)
        if input_type is list:
            length = len(input)
            if length == 0:
                rospy.logerr("convert type failed...input is " + input)
                return 0
            else:
                arr = (ctypes_map[type(input[0])] * length)()
                for i in range(length):
                    arr[i] = bytes(
                        input[i], encoding="utf-8") if (type(input[0]) is str) else input[i]
                return arr
        else:
            if input_type in ctypes_map:
                return ctypes_map[input_type](bytes(input, encoding="utf-8") if type(input) is str else input)
            else:
                rospy.logerr("convert type failed...input is "+input)
                return 0

    def __thread_job(self):
        rospy.spin()

    def __callback_gait(self, req):
        self.cpp_gait_ctrller.set_gait_type(self.__convert_type(req.cmd))
        return QuadrupedCmdResponse(0, "get the gait")

    def __callback_mode(self, req):
        self.cpp_gait_ctrller.set_robot_mode(self.__convert_type(req.cmd))
        return QuadrupedCmdResponse(0, "get the mode")

    def __callback_body_vel(self, msg):
        vel = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.cpp_gait_ctrller.set_robot_vel(self.__convert_type(vel))

    def __fill_tf_message(self, parent_frame, child_frame, translation, rotation):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        return t

    def __pub_nav_msg(self, base_pos, imu_data):
        pub_odom = rospy.Publisher("/robot_odom", Odometry, queue_size=30)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"
        odom.child_frame_id = "body"
        odom.pose.pose.position.x = base_pos[0]
        odom.pose.pose.position.y = base_pos[1]
        odom.pose.pose.position.z = base_pos[2]
        odom.pose.pose.orientation.x = imu_data[3]
        odom.pose.pose.orientation.y = imu_data[4]
        odom.pose.pose.orientation.z = imu_data[5]
        odom.pose.pose.orientation.w = imu_data[6]

        pub_odom.publish(odom)

        # Publish odom Tf
        t = self.__fill_tf_message(
            odom.header.frame_id, odom.child_frame_id, base_pos[0:3], imu_data[3:7])
        self.robot_tf.sendTransform(t)

    def __pub_ground_truth_pose(self, base_pos, imu_data):
        pub_gt_pose = rospy.Publisher("/gt_pose", PoseWithCovarianceStamped, queue_size=1)
        gt_pose = PoseWithCovarianceStamped()
        gt_pose.header.stamp = rospy.Time.now()
        gt_pose.header.frame_id = "body"
        gt_pose.pose.pose.position.x = base_pos[0]
        gt_pose.pose.pose.position.y = base_pos[1]
        gt_pose.pose.pose.position.z = base_pos[2]
        gt_pose.pose.pose.orientation.x = imu_data[3]
        gt_pose.pose.pose.orientation.y = imu_data[4]
        gt_pose.pose.pose.orientation.z = imu_data[5]
        gt_pose.pose.pose.orientation.w = imu_data[6]
        pub_gt_pose.publish(gt_pose)

    def __pub_imu_msg(self, imu_data):
        pub_imu = rospy.Publisher("/imu0", Imu, queue_size=30)
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = imu_data[0]
        imu_msg.linear_acceleration.y = imu_data[1]
        imu_msg.linear_acceleration.z = imu_data[2]
        imu_msg.angular_velocity.x = imu_data[7]
        imu_msg.angular_velocity.y = imu_data[8]
        imu_msg.angular_velocity.z = imu_data[9]
        imu_msg.orientation.x = imu_data[3]
        imu_msg.orientation.y = imu_data[4]
        imu_msg.orientation.z = imu_data[5]
        imu_msg.orientation.w = imu_data[6]
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "body"
        pub_imu.publish(imu_msg)

    def __pub_joint_states(self, joint_states):
        pub_js = rospy.Publisher("joint_states", JointState, queue_size=30)
        js_msg = JointState()
        js_msg.name = []
        js_msg.position = []
        js_msg.velocity = []
        for idx, name in enumerate(joint_states["name"]):
            js_msg.name.append(name.decode('utf-8'))
            js_msg.position.append(joint_states["state"][idx])
            js_msg.velocity.append(joint_states["state"][12+idx])
        js_msg.header.stamp = rospy.Time.now()
        js_msg.header.frame_id = "body"
        pub_js.publish(js_msg)

    def __pub_whole_body_state(self, imu_data, leg_data, base_pos, contact_points):
        wbs_pub = rospy.Publisher("wb_state", WholeBodyState, queue_size=10)
        wbs = WholeBodyState()
        wbs.header.stamp = rospy.Time.now()
        wbs.header.frame_id = "world"
        wbs.time = wbs.header.stamp.secs
        # This represents the base state (CoM motion, angular motion and centroidal momenta)
        wbs.centroidal.com_position.x = base_pos[0]
        wbs.centroidal.com_position.y = base_pos[1]
        wbs.centroidal.com_position.z = base_pos[2]
        wbs.centroidal.base_orientation.x = imu_data[3]
        wbs.centroidal.base_orientation.y = imu_data[4]
        wbs.centroidal.base_orientation.z = imu_data[5]
        wbs.centroidal.base_orientation.w = imu_data[6]
        wbs.centroidal.base_angular_velocity.x = imu_data[7]
        wbs.centroidal.base_angular_velocity.y = imu_data[8]
        wbs.centroidal.base_angular_velocity.z = imu_data[9]
        # This represents the joint state (position, velocity, acceleration and effort)
        wbs.joints = []
        for idx, name in enumerate(leg_data["name"]):
            js_msg = WBJointState()
            js_msg.name = name.decode('utf-8')
            js_msg.position = leg_data["state"][idx]
            js_msg.velocity = leg_data["state"][12+idx]
            wbs.joints.append(js_msg)
        # This represents the end-effector state (cartesian position and contact forces)
        wbs.contacts = []
        for contact_point in contact_points:
            contact_msg = WBContactState()
            contact_msg.name = "body"
            contact_msg.type = WBContactState.ACTIVE
            contact_msg.pose.position.x = contact_point[5][0]
            contact_msg.pose.position.y = contact_point[5][1]
            contact_msg.pose.position.z = contact_point[5][2]
            contact_msg.wrench.force.z = contact_point[9]
            contact_msg.surface_normal.x = contact_point[7][0]
            contact_msg.surface_normal.y = contact_point[7][1]
            contact_msg.surface_normal.z = contact_point[7][2]
            contact_msg.friction_coefficient = self.lateralFriction
            wbs.contacts.append(contact_msg)
        wbs_pub.publish(wbs)

    def __get_motor_joint_states(self, robot):
        joint_number_range = range(p.getNumJoints(robot))
        joint_states = p.getJointStates(robot, joint_number_range)
        joint_infos = [p.getJointInfo(robot, i) for i in joint_number_range]
        joint_states, joint_name = \
            zip(*[(j, i[1]) for j, i in zip(joint_states, joint_infos) if i[2] != p.JOINT_FIXED])
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques, joint_name

    def __get_data_from_sim(self):
        get_matrix = []
        get_velocity = []
        get_invert = []
        imu_data = [0] * 10
        leg_data = {}
        leg_data["state"] = [0] * 24
        leg_data["name"] = [""] * 12

        base_pose = p.getBasePositionAndOrientation(self.boxId)

        get_velocity = p.getBaseVelocity(self.boxId)
        get_invert = p.invertTransform(base_pose[0], base_pose[1])
        get_matrix = p.getMatrixFromQuaternion(get_invert[1])

        # IMU data
        imu_data[3] = base_pose[1][0]
        imu_data[4] = base_pose[1][1]
        imu_data[5] = base_pose[1][2]
        imu_data[6] = base_pose[1][3]

        imu_data[7] = get_matrix[0] * get_velocity[1][0] + get_matrix[1] * \
            get_velocity[1][1] + get_matrix[2] * get_velocity[1][2]
        imu_data[8] = get_matrix[3] * get_velocity[1][0] + get_matrix[4] * \
            get_velocity[1][1] + get_matrix[5] * get_velocity[1][2]
        imu_data[9] = get_matrix[6] * get_velocity[1][0] + get_matrix[7] * \
            get_velocity[1][1] + get_matrix[8] * get_velocity[1][2]

        # calculate the acceleration of the robot
        linear_X = (get_velocity[0][0] - self.get_last_vel[0]) * self.freq
        linear_Y = (get_velocity[0][1] - self.get_last_vel[1]) * self.freq
        linear_Z = 9.8 + (get_velocity[0][2] - self.get_last_vel[2]) * self.freq
        imu_data[0] = get_matrix[0] * linear_X + \
            get_matrix[1] * linear_Y + get_matrix[2] * linear_Z
        imu_data[1] = get_matrix[3] * linear_X + \
            get_matrix[4] * linear_Y + get_matrix[5] * linear_Z
        imu_data[2] = get_matrix[6] * linear_X + \
            get_matrix[7] * linear_Y + get_matrix[8] * linear_Z

        # joint data
        joint_positions, joint_velocities, _, joint_names = \
            self.__get_motor_joint_states(self.boxId)
        leg_data["state"][0:12] = joint_positions
        leg_data["state"][12:24] = joint_velocities
        leg_data["name"] = joint_names

        # CoM velocity
        self.get_last_vel = [get_velocity[0][0], get_velocity[0][1], get_velocity[0][2]]

        # Contacts
        contact_points = p.getContactPoints(self.boxId)

        return imu_data, leg_data, base_pose[0], contact_points

    def __callback_elevation_map(self, msg):
        map = list(msg.data[0].data)
        self.cpp_gait_ctrller.store_map(self.__convert_type(map))


if __name__ == '__main__':
    rospy.init_node('quadruped_simulator', anonymous=True)
    walking_simulation = WalkingSimulation()
    walking_simulation.run()
