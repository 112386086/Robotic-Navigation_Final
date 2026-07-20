from rclpy.node import Node
from pros_car_py.car_models import DeviceDataTypeEnum, CarCControl
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, PointStamped
from std_msgs.msg import String, Header
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, Imu
from trajectory_msgs.msg import JointTrajectoryPoint
import orjson
from pros_car_py.ros_communicator_config import ACTION_MAPPINGS
from std_msgs.msg import Bool, Float32MultiArray
from visualization_msgs.msg import Marker
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import rclpy
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

#from rclpy.qos import QoSPresetProfiles
class RosCommunicator(Node):
    def __init__(self):
        super().__init__("RosCommunicator")

        #tf_buffer用來儲存tf2的轉換資訊
        self.tf_buffer = tf2_ros.Buffer() 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # subscribe map接收地圖
        self.latest_map = None
        # QoS設定，後續create_subscription時直接把qos_profile作為參數傳入
        map_qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 持久化策略
            reliability=ReliabilityPolicy.RELIABLE,  # 確保可靠傳輸
            history=HistoryPolicy.KEEP_LAST,  # 保留最新的消息
        )
        self.subscriber_map = self.create_subscription(
            OccupancyGrid, "/map", self.subscriber_map_callback, 
            map_qos_profile
        )
        
        # QoS profile for /amcl_pose, 
        amcl_qos_profile = QoSProfile(
            depth=10,  # 深度設定
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  
            reliability=ReliabilityPolicy.RELIABLE,  
            history=HistoryPolicy.KEEP_LAST  
        )
        
        # subscribe amcl_pose,車輛在地圖的pose&位置
        self.latest_amcl_pose = None
        self.subscriber_amcl = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.subscriber_amcl_callback, amcl_qos_profile
        )

        # QoS profile for /odom, 
        odom_qos_profile = QoSProfile(
            depth=10,  # 深度設定
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,  
            history=HistoryPolicy.KEEP_LAST  
        )

        # subscribe odom,車輛在地圖的pose&位置
        self.latest_odom = None
        self.subscriber_odom = self.create_subscription(
            Odometry, "/odom", self.subscriber_odom_callback, odom_qos_profile
        )

    
        # QoS profile for goal pose
        goal_qos_profile = QoSProfile(
            depth=10, 
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # subscribe /goal_pose
        self.target_pose = None
        self.subscriber_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self.subscriber_goal_callback, goal_qos_profile
        )

        # QoS profile for /scan,
        scan_qos_profile = QoSProfile(
            depth = 10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # subscribe lidar,訂閱/scan接收lidar數據
        self.latest_lidar = None
        self.subscriber_lidar = self.create_subscription( 
            LaserScan, "/scan", self.subscriber_lidar_callback, scan_qos_profile
        )


        # subscribe global_plan,接收全局路徑
        self.latest_received_global_plan = None
        self.subscriber_received_global_plan = self.create_subscription(
            Path, "/received_global_plan", self.received_global_plan_callback, 1
        )

        # Subscribe to YOLO detected object coordinates
        self.latest_yolo_position = None
        self.subscriber_yolo_detection_position = self.create_subscription(
            PointStamped,
            "/yolo/detection/position",
            self.yolo_detection_position_callback,
            10,
        )

        # Subscribe to YOLO detected object coordinates,物體相對於參考點的偏移量
        self.latest_yolo_offset = None
        self.subscriber_yolo_offset = self.create_subscription(
            PointStamped,
            "/yolo/detection/offset",
            self.yolo_detection_offset_callback,
            10,
        )

        # Subscribe to YOLO detection status,yolo是否有偵測到物體
        self.latest_yolo_detection_status = None
        self.subscriber_yolo_detection_status = self.create_subscription(
            Bool, "/yolo/detection/status", self.yolo_detection_status_callback, 10
        )
        
        # QoS profile for target label
        target_label_qos_profile = QoSProfile(
            depth = 1,
            durability=DurabilityPolicy.VOLATILE,  
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # ## Subscribe to Yolo target label,接收YOLO目標標籤
        self.latest_yolo_target_label_msg = None
        self.subscriber_yolo_target_label = self.create_subscription(
            String,
            "/target_label",
            self.yolo_target_label_callback,
            target_label_qos_profile
        )

        # Subscribe to IMU data,接收IMU數據
        self.latest_imu_data = None
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self.imu_data_callback, 10
        )

        self.latest_mediapipe_data = None
        self.mediapipe_sub = self.create_subscription(
            Point, "/mediapipe_data", self.mediapipe_data_callback, 10
        )

        # Subscribe to YOLO target info,接收YOLO目標詳細資訊
        self.latest_yolo_target_info = None
        self.yolo_target_info_sub = self.create_subscription(
            Float32MultiArray, "/yolo/target_info", self.yolo_target_info_callback, 10
        )

        # Subscribe to camera x_multi_depth values,接收相機多深度值
        self.latest_camera_x_multi_depth = None
        self.camera_x_multi_depth_sub = self.create_subscription(
            Float32MultiArray,
            "/camera/x_multi_depth_values",
            self.camera_x_multi_depth_callback,
            15, 
        )

        # publish car_C_rear_wheel and car_C_front_wheel
        self.publisher_rear = self.create_publisher(
            Float32MultiArray, DeviceDataTypeEnum.car_C_rear_wheel, 10
        )
        self.publisher_forward = self.create_publisher(
            Float32MultiArray, DeviceDataTypeEnum.car_C_front_wheel, 10
        )
        
        # publish initial pose,發布初始位置
        self.publisher_init_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        # publish goal_pose,發布目標位置
        self.publisher_goal_pose = self.create_publisher(
            PoseStamped, "/goal_pose", goal_qos_profile)

        # publish robot arm angle
        self.publisher_joint_trajectory = self.create_publisher(
            JointTrajectoryPoint, DeviceDataTypeEnum.robot_arm, 10
        )

        self.publisher_coordinates = self.create_publisher(
            PointStamped, "/coordinates", 10
        )
        # create publisher of target label
        self.publisher_target_label = self.create_publisher(String, "/target_label", 10)

        self.crane_state_publisher = self.create_publisher(String, "crane_state", 10)

        self.publisher_confirmed_path = self.create_publisher(
            Path, "/confirmed_initial_plan", 10
        )

        self.publisher_target_marker = self.create_publisher(
            Marker, "/selected_target_marker", 10
        )

        # 清除 costmap Service
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear"
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear"
        )

        self.publisher_received_global_plan = self.create_publisher(
            Path, "/received_global_plan", 10
        )
        self.publisher_plan = self.create_publisher(Path, "/plan", 10)

        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear"
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear"
        )

        self.navigate_to_pose_action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )

    def clear_received_global_plan(self):
        """
        清空 /received_global_plan 话题
        """
        empty_path = Path()
        empty_path.header.frame_id = "map"
        self.publisher_received_global_plan.publish(empty_path)
        self.get_logger().info("Published empty Path to /received_global_plan")

    def clear_plan(self):
        """
        清空 /plan 话题
        """
        empty_path = Path()
        empty_path.header.frame_id = "map"
        self.publisher_plan.publish(empty_path)
        self.get_logger().info("Published empty Path to /plan")

    def reset_nav2(self):
        """
        clear plan
        """
        self.clear_received_global_plan()
        self.clear_plan()
        self.get_logger().info("Nav2 Reset Completed")

    # map callback
    def subscriber_map_callback(self, msg):
        self.latest_map = msg

    def get_latest_map(self):
        if self.latest_map is None:
            self.get_logger().warn("No map data received yet.")
            return None
        return self.latest_map


    # amcl_pose callback and get_latest_amcl_pose
    def subscriber_amcl_callback(self, msg):
        if msg.header.frame_id != "map":
            self.get_logger().warn("AMCL pose frame_id is not 'map'.")
        self.latest_amcl_pose = msg
        

    def get_latest_amcl_pose(self):
        if self.latest_amcl_pose is None:
            self.get_logger().warn("No AMCL pose data received yet.")
        return self.latest_amcl_pose

    # odom callback and get_latest_odom
    def subscriber_odom_callback(self, msg):
        self.latest_odom = msg
    
    def get_latest_odom(self):
        if self.latest_odom is None:
            self.get_logger().warn("No odom data received yet.")
            return None
        return self.latest_odom

    # goal callback and get_latest_goal
    def subscriber_goal_callback(self, msg):
        position = msg.pose.position
        target = [position.x, position.y, position.z]
        self.target_pose = target

    def get_latest_goal(self):
        if self.target_pose is None:
            self.get_logger().warn("No goal pose data received yet.")
        return self.target_pose

    # lidar callback and get_latest_lidar
    def subscriber_lidar_callback(self, msg):
        self.latest_lidar = msg

    def get_latest_lidar(self):
        if self.latest_lidar is None:
            self.get_logger().warn("No Lidar data received yet.")
        return self.latest_lidar

    # received_global_plan callback and get_latest_received_global_plan
    def received_global_plan_callback(self, msg):
        self.latest_received_global_plan = msg

    def get_latest_received_global_plan(self):
        if self.latest_received_global_plan is None:
            self.get_logger().warn("No received global plan data received yet.")
            return None
        return self.latest_received_global_plan

    def publish_car_control(self, action_key, publish_rear=True, publish_front=True):
        msg = Float32MultiArray()
        if action_key not in ACTION_MAPPINGS:
            # print("action error")
            return
        velocities = ACTION_MAPPINGS[action_key]
        self._vel1, self._vel2, self._vel3, self._vel4 = velocities
        msg.data = [self._vel1, self._vel2]
        if publish_rear == True:
            self.publisher_rear.publish(msg)
        msg.data = [self._vel3, self._vel4]
        if publish_front == True:
            self.publisher_forward.publish(msg)
    

    def get_map_basefootprint(self):
        """
        使用tf2取得odom到base_footprint的轉換，包含hearder和transform(translation和rotation)
        """
        try: #lookup_transform要變換可用時才不會trigger exception
            trans_stamp = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return trans_stamp 
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return None

    # publish goal_pose
    def publish_goal_pose(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header = Header()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.publisher_goal_pose.publish(goal_pose)

    # publish init pose
    def publish_init_pose(self): 
        init_pose = PoseWithCovarianceStamped()
        init_pose.header = Header()
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.header.frame_id = "map"
        # 設定初始位置和方向
        init_pose.pose.pose.position.x = 0.005208466109470011
        init_pose.pose.pose.position.y = -0.0014680537013971318  
        init_pose.pose.pose.position.z = 0.0
        init_pose.pose.pose.orientation.x = 0.0
        init_pose.pose.pose.orientation.y = 0.0
        init_pose.pose.pose.orientation.z = -0.009583266743157072
        init_pose.pose.pose.orientation.w = 0.9999540794449161 
        # set covariance 
        init_pose.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        self.publisher_init_pose.publish(init_pose)

    # publish robot arm angle
    def publish_robot_arm_angle(self, angle):
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = angle
        joint_trajectory_point.velocities = [0.0] * len(angle)
        self.publisher_joint_trajectory.publish(joint_trajectory_point)

    def publish_coordinates(self, x, y, z, frame_id="map"):
        coordinate_msg = PointStamped()
        coordinate_msg.header.stamp = self.get_clock().now().to_msg()
        coordinate_msg.header.frame_id = frame_id
        coordinate_msg.point.x = x
        coordinate_msg.point.y = y
        coordinate_msg.point.z = z
        self.publisher_coordinates.publish(coordinate_msg)

    def mediapipe_data_callback(self, msg):
        self.latest_mediapipe_data = msg

    def get_latest_mediapipe_data(self):
        if self.latest_mediapipe_data is None:
            self.get_logger().warn("No Mediapipe data received yet.")
            return None
        return self.latest_mediapipe_data

    def yolo_target_info_callback(self, msg):
        self.latest_yolo_target_info = msg

    def get_latest_yolo_target_info(self):
        if self.latest_yolo_target_info is None:
            return None
        return self.latest_yolo_target_info
    
    def yolo_target_label_callback(self, msg):
        print("yolo_target_label_callback", msg.data)
        self.latest_yolo_target_label = msg

    def get_latest_yolo_target_label(self):
        if self.latest_yolo_target_label is None:
            return None
        return self.latest_yolo_target_label

    def camera_x_multi_depth_callback(self, msg):
        self.latest_camera_x_multi_depth = msg

    def get_latest_camera_x_multi_depth(self):
        if self.latest_camera_x_multi_depth is None:
            return None
        return self.latest_camera_x_multi_depth

    # YOLO coordinates callback
    def yolo_detection_position_callback(self, msg):
        """Callback to receive YOLO detected object coordinates."""
        self.latest_yolo_position = msg

    def get_latest_yolo_detection_position(self):
        """Getter for the latest YOLO detected object coordinates."""
        if self.latest_yolo_position is None:
            return None
        return self.latest_yolo_position

    def yolo_detection_offset_callback(self, msg):
        self.latest_yolo_offset = msg

    def get_latest_yolo_detection_offset(self):
        if self.latest_yolo_offset is None:
            return None
        return self.latest_yolo_offset

    def publish_target_label(self, label):
        target_label_msg = String() 
        target_label_msg.data = label
        print("publish_target_label", target_label_msg.data)
        self.publisher_target_label.publish(target_label_msg)

    


    # 天車
    def publish_crane_state(self, state):
        control_signal = {"type": "crane", "data": dict(crane_state=state)}
        crane_state_msg = String()
        crane_state_msg.data = orjson.dumps(control_signal).decode()
        self.crane_state_publisher.publish(crane_state_msg)

    def yolo_detection_status_callback(self, msg):
        self.latest_yolo_detection_status = msg

    def get_latest_yolo_detection_status(self):
        if self.latest_yolo_detection_status is None:
            return None
        return self.latest_yolo_detection_status

    def imu_data_callback(self, msg):
        self.latest_imu_data = msg

    def get_latest_imu_data(self):
        if self.latest_imu_data is None:
            return None
        return self.latest_imu_data

    def publish_confirmed_initial_plan(self, path_msg: Path):
        """
        確認路徑使用
        """
        self.publisher_confirmed_path.publish(path_msg)

    def publish_selected_target_marker(self, x, y, z=0.0):
        """
        在 foxglove 畫紅點
        """
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.2  # 球體大小
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # 透明度
        marker.color.r = 1.0  # 顏色
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.publisher_target_marker.publish(marker)
