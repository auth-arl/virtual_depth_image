import cv_bridge
import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration

import transforms3d
import numpy as np
import cv2

from .VirtualDepthCamera import VirtualDepthCamera

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from proghrc_msgs.msg import SimpleCameraInfo
from proghrc_msgs.msg import NodeHealth

from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data

import time
from sensor_msgs.msg import Image



def _from_ROS(tf_):
    t = np.array([tf_.transform.translation.x,
                  tf_.transform.translation.y,
                  tf_.transform.translation.z])
    q = np.array([tf_.transform.rotation.w,
                  tf_.transform.rotation.x,
                  tf_.transform.rotation.y,
                  tf_.transform.rotation.z,
                  ])
    R = transforms3d.quaternions.quat2mat(q)
    model_tf = np.eye(4)
    model_tf[0:3, 0:3] = R[:3, :3]
    model_tf[0:3, 3] = t[:3]
    return model_tf


class VirtualDepthCamera_Node(Node):

    def __init__(self):
        super().__init__('Virtual_Depth_Node')

        self.declare_parameter("mode", "MASK")
        self.declare_parameter("viz_enabled", False)
        self.declare_parameter("camera_prefix", "")
        self.declare_parameter("scaling", 1.0)
        # self.declare_parameter("camera_frame", "camera_conveyor")
        self.declare_parameter("downscale_factor", 1.0,
                               ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                   description="mask will have downsized size of original_size/downsize_factor"))
        # Virtual Depth Camera
        self.virtualDepthCamera = None # object will be constructed by Init_callback when params are retrieved from other nodes.

        # VDC params:
        # self.camera_frame   =  self.get_parameter("camera_frame").value
        self.camera_frame = None

        self.camera_info_msg    = None # to be initialized by  "camera_info_callback" subscriber callback
        self.robot_description  = None # to be retrieved by service client from robot_state_publisher node parameters
        self.cv_virtual_image = None # output

        # ROS2 Comm Utils
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self,qos = qos_profile_sensor_data)
        self.cv_bridge = cv_bridge.CvBridge()

        # camera prefix.
        camera_prefix = self.get_parameter("camera_prefix").value
        self.mode = self.get_parameter("mode").value

        qos = qos_profile_sensor_data
        # qos =  5
        self.camera_info_sub    = self.create_subscription(SimpleCameraInfo, camera_prefix + "camera/info", self.camera_info_callback, qos_profile=qos)
        self.mask_image_pub     = self.create_publisher(Image, camera_prefix + "camera/occlusion_mask", qos_profile=5)

        viz_enabled = self.get_parameter("viz_enabled").value
        if viz_enabled:
            self.color_sub = self.create_subscription(Image, camera_prefix + "camera/color", self.viz_callback, qos_profile=qos)
            self.composite_pub = self.create_publisher(Image, camera_prefix + "camera/virtual/composite", qos_profile=5)

        #  this service client retrieces robot_description param from robot_state_publisher.
        self.cli = self.create_client(GetParameters, 'robot_state_publisher/get_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Waiting for ''robot_state_publisher/get_parameters'' Service. Make sure that arl_description is launched and running normally')
        
        time.sleep(1)
        self.req = GetParameters.Request()
        self.req.names = ["robot_description"]
        self.future = self.cli.call_async(self.req)
        print("Requested robot description.")

        # ROS2 Control Flow: Timers and flags
        self.init_timer = self.create_timer(0.50, self.init_callback)
        self.main_timer = None # this will be created when init callback decides so

        # control flow flags
        self.camera_info_ready_FLAG = False
        self.robot_description_ready_FLAG = False
        self.init_steps_counter = 0 #

        # Node health publisher
        self.h_count = 0
        self.health_publisher = self.create_publisher(NodeHealth, '/proghrc/node_health',10)   
        self.alive_msg = NodeHealth()
        self.alive_msg.node_name = self.get_name()   

    def camera_info_callback(self, msg: SimpleCameraInfo):
        # When the camera info is retrieved, this subscriber callback self-destructs
        self.camera_info_msg = msg
        self.camera_info_ready_FLAG = True
        self.print_initialization_progress("Camera Info retrieved from topic")
        # print(msg)
        self.destroy_subscription(self.camera_info_sub)

    def init_callback(self):
        # Setting up the Virtual Depth Camera Object.
        #
        # Init callback will actually execute when:
        #   a) the camera_info is retrieved from the appropriate topic
        #   b) the robot description is parsed
        #
        # After that, this callback method destroys its timer and initialized the main_timer
        #   that calls the main_loop of the system

        # poll for the service async call.
        if self.robot_description_ready_FLAG == False:
            if  self.future.done() == True:
                self.robot_description = self.future.result().values[0].string_value
                self.robot_description_ready_FLAG = True
                self.print_initialization_progress("Robot Description Received from robot_state_publisher")
            else:
                print("Waiting for description service response.")
        


        if (self.camera_info_ready_FLAG and self.robot_description_ready_FLAG):
            print("")
            print("-------- Virtual Depth Camera Start --------------")

            # construct VirtualDepthCamera obj
            from ament_index_python.packages import get_package_share_directory
            ur_desc_folder_path = get_package_share_directory('arl_description')

            downscale_factor = self.get_parameter("downscale_factor").value
            camera_params_ = [self.camera_info_msg.fx/downscale_factor,
                              self.camera_info_msg.fy/downscale_factor,
                              self.camera_info_msg.cx/downscale_factor,
                              self.camera_info_msg.cy/downscale_factor]
            self.virtualDepthCamera = VirtualDepthCamera(self.get_parameter("mode").value,
                                                         self.robot_description,
                                                         ur_desc_folder_path,
                                                         camera_params_,
                                                         int(self.camera_info_msg.color_width/downscale_factor),
                                                         int(self.camera_info_msg.color_height/downscale_factor),
                                                         self.get_parameter("scaling").value)
            self.print_robot_links()
            print("-------- Virtual Depth Camera Start --------------")
            print("---  MODE : ",self.get_parameter("mode").value,"  ---")
            self.camera_frame = self.camera_info_msg.frame_id


            # create main timer with the actual callback, and destroy the init_callback timer
            # self.main_timer.timer_period_ns = Duration(1/self.camera_info_msg.fps).nanoseconds
            self.create_timer(1/self.camera_info_msg.fps, self.run_callback)
            self.destroy_timer(self.init_timer)

        else:
            pass

    def print_robot_links(self):
        print("")
        print(" The Occlusion Mask will consist of the following links from the URDF file:")
        print("   |   ")
        for link_ in self.virtualDepthCamera.get_tf_link_names():
            print("   |   " + link_)
        print("   |   ")
        print("   |_______________________________________")
        print("")

    def run_callback(self):
        
        # st_ = self.get_clock().now()
        if self.virtualDepthCamera is not None:
            timeout_ = Duration(seconds=0.01, nanoseconds=0)
            robot_joints_tf_list = []
            for link_ in self.virtualDepthCamera.get_tf_link_names():
                try:
                    tf_ = self.tf_buffer.lookup_transform(
                        self.camera_frame,
                        link_,
                        Time(seconds=0, nanoseconds=0),  # give it Time() -> sec=0, nsec=0 to yield latest tf
                        timeout=timeout_)                 
                    

                    robot_joints_tf_list.append(_from_ROS(tf_))
                except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                    print('FAILED TO RETRIEVE TF OF LINK: ' + link_+" from : " + self.camera_frame )
                    robot_joints_tf_list.append(np.eye(4))
            # st_ = self.get_clock().now()
            self.virtualDepthCamera.run(robot_joints_tf_list)
            if(self.mode == "DEPTH"):
                self.cv_virtual_image = self.virtualDepthCamera.get_depth_image()

                img_msg = self.cv_bridge.cv2_to_imgmsg(self.cv_virtual_image, encoding="32FC1")
                img_msg.header.frame_id = self.camera_frame
                self.mask_image_pub.publish(img_msg)


            elif(self.mode == "MASK"):
                self.cv_virtual_image = self.virtualDepthCamera.get_mask_image()

                img_msg = self.cv_bridge.cv2_to_imgmsg(self.cv_virtual_image, encoding="8UC1")
                img_msg.header.frame_id = self.camera_frame
                self.mask_image_pub.publish(img_msg)
            else:
                print("INVALID MODE. exiting...\n\n")
                exit(-1)
            # et_ = self.get_clock().now()

            # print((et_.nanoseconds-st_.nanoseconds)/1000000.0)
            # print(self.main_timer.time_until_next_call()/1000000.0)

            # cv2.imshow("virtual", virtual_image)
            # cv2.waitKey(5)
            self.sendAlive()
           
    def print_initialization_progress(self, text_msg):
        if self.init_steps_counter == 0:
            # fancy-pantsy console viz
            print("---------------INITIALIZATION---------------------")
        self.init_steps_counter = self.init_steps_counter + 1
        print("INIT PROGRESS: " + str(self.init_steps_counter) + "/2 - " + text_msg)
        if self.init_steps_counter == 2:
            # fancy-pantsy console viz
            print("---------------INITIALIZATION---------------------")

    def viz_callback(self, msg : Image):
        """
            This method callback is enabled only if viz_enabled is True.

            As it receives color image from camera, it creates a composite image of the rgb and the mask and publishes it in composite_pup
        :param msg:
        :return: None
        """

        if self.cv_virtual_image is None: # first image not yet generated
            return
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        
        if self.mode == "DEPTH":
            max_tmp = np.max(self.cv_virtual_image)
            mask_red = self.cv_virtual_image
            mask_red[np.nonzero(self.cv_virtual_image == max_tmp)] = 0
            mask_red = cv2.normalize(mask_red, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
            mask_red = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2RGB)
        else:  
            mask_red = cv2.cvtColor(self.cv_virtual_image, cv2.COLOR_GRAY2RGB)

        # zero the Blue and Green channels: keep only the Red
        mask_red[::, ::, [0, 1]] = 0
        dsize = cv_image.shape
        mask_red = cv2.resize(mask_red, (dsize[1], dsize[0]))
        # Superimpose the camera image with the virtual one
        cv_composite = cv2.addWeighted(cv_image, 1.0, mask_red, 0.5, 0)


        self.composite_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_composite,"bgr8"))




    def destroy_node(self) -> bool:
        if self.virtualDepthCamera is not None:
            self.virtualDepthCamera.destroy_virtual_camera()
        super().destroy_node()

    # publish send alive every 2 sec 
    def sendAlive(self):
        if( self.h_count == 2 * self.camera_info_msg.fps ):            
            self.alive_msg.stamp = self.get_clock().now().to_msg()
            self.health_publisher.publish(self.alive_msg)
            self.h_count = 0
        self.h_count = self.h_count + 1



def main(args=None):
    rclpy.init(args=args)
    print("Starting Virtual Depth Camera Node: ")

    # Add imported nodes to this executor
    try:
        node = VirtualDepthCamera_Node()
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        print("Exiting Node...")
        node.destroy_node()
        rclpy.try_shutdown()
    except RuntimeError as e:
        node.destroy_node()
        rclpy.try_shutdown()
        raise e
    return 0


if __name__ == '__main__':
    main()
