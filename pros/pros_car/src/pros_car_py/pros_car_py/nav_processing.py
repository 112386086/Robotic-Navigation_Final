from pros_car_py.nav2_utils import (
    get_yaw_from_quaternion,
    get_direction_vector,
    get_angle_to_target,
    calculate_angle_point,
    cal_distance,
)
import math
import numpy as np
import random
from geometry_msgs.msg import PoseStamped
from pros_car_py.car_controller import CarController
from pros_car_py.nav2_utils import (
    calculate_angle_to_target, cal_distance, calculate_angle_point)

class Nav2Processing:
    def __init__(self, ros_communicator, data_processor):
        self.ros_communicator = ros_communicator
        self.data_processor = data_processor
        self.finishFlag = False
        self.global_plan_msg = None
        self.index = 0 #路徑點index
        self.index_length = 0
        self.recordFlag = 0 #紀錄是否已visited過路徑點
        self.goal_published_flag = False
        # init map data
        self.map_data = None
        self.map_info = None
        self.visited_map = None  
        #
        self.current_explor_target = None  # 用於存儲當前探索目標


    def reset_nav_process(self):
        self.finishFlag = False
        self.recordFlag = 0
        self.goal_published_flag = False
    
    def finish_nav_process(self):
        self.finishFlag = True
        self.recordFlag = 1

    def get_finish_flag(self):
        return self.finishFlag
    
    def world_to_grid_coordinates(self, world_x, world_y, map_info):
        if map_info is None:
            #沒有map.info沒得轉換
            print("no map data exists!")
            return None, None #因為是return map_x, map_y
        grid_x = int((world_x - map_info.origin.position.x) / map_info.resolution)
        grid_y = int((world_y - map_info.origin.position.y) / map_info.resolution)

        if not (0 <= grid_x < map_info.width and 0 <= grid_y < map_info.height):
            self.ros_communicator.get_logger().warn(
                f"World coordinates ({world_x}, {world_y}) are out of map bounds."
            )
            return None, None
        return grid_y, grid_x  # map_data是row major, 所以返回順序是 (y, x)
    
    def grid_to_world_coordinates(self, grid_x, grid_y, map_info):
        """將柵格座標轉換為世界座標"""
        # 確保grid_x, grid_y is int or float
        grid_x = int(grid_x)
        grid_y = int(grid_y)
        world_x = map_info.origin.position.x + (grid_x + 0.5) * map_info.resolution
        world_y = map_info.origin.position.y + (grid_y + 0.5) * map_info.resolution
        # +0.5 是為了取格子的中心點
        return world_x, world_y
    
    # 找到最近的未知領域
    def find_closest_frontier_point(self, car_position_world, map_data, map_info):
        
        if map_data is None or map_info is None or car_position_world is None:
            return None
        # np.where will 2D array, (row_indices, col_indices)
        unknown_indices_rows, unknown_indices_cols = np.where(map_data == -1) # (row_indices, col_indices)
        if unknown_indices_rows.size == 0:
            # current map are all known, 改找自由空間探索
            self.ros_communicator.get_logger().info("No unknown areas, find free space.")
            free_indices_rows, free_indices_cols = np.where(map_data == 0) # 0 代表自由空間
            if free_indices_rows.size == 0:
                self.ros_communicator.get_logger().warn("No free space found in the map.")
                return None
            idx = random.randint(0, free_indices_rows.size - 1) # 隨機選擇一個自由空間
            grid_y, grid_x = free_indices_rows[idx], free_indices_cols[idx]
            print(f"[DEBUG] free_indices_rows: {free_indices_rows}, free_indices_cols: {free_indices_cols}")
            print(f"[DEBUG] idx: {idx}, grid_x: {grid_x}, grid_y: {grid_y}")
            print(f"[DEBUG] map_info: origin=({map_info.origin.position.x}, {map_info.origin.position.y}), resolution={map_info.resolution}")
            return self.grid_to_world_coordinates(grid_x, grid_y, map_info)

        # 讀取車輛位置轉換到grid座標
        car_grid_y, car_grid_x = self.world_to_grid_coordinates(
            car_position_world[0], car_position_world[1], map_info
        )

        min_dist_sq = float('inf') #初始化令無窮大
        closest_frontier_grid = None

        # 取出check_node個未知點then隨機取點
        check_node = min(5, unknown_indices_rows.size) # 最多檢查100個點
        random_sele = random.sample(range(unknown_indices_rows.size), check_node)
        # 遍歷上述挑到的未知區域
        for i in random_sele:
            # get unknown area grid coordinates
            grid_y, grid_x = unknown_indices_rows[i], unknown_indices_cols[i]
            is_frontier = False
            # 檢查未知區域是否為障礙物
            if map_data[grid_y, grid_x] == 100:
                continue

            # 檢查在 visited_map 中是否已經被標記過
            if (self.visited_map is not None and (self.visited_map[grid_y, grid_x] == 1) ):
                continue
            
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dx == 0 and dy == 0: 
                        #
                        continue
                    adj_y, adj_x = grid_y + dy, grid_x + dx
                    if 0 <= adj_y < map_info.height and 0 <= adj_x < map_info.width:
                        if map_data[adj_y, adj_x] == 0: # 代表自由空間
                            is_frontier = True
                            break
                if is_frontier is True:
                    break
            # 找到離車輛最近的未知區域
            if is_frontier:
                dist_sq = (grid_x - car_grid_x)**2 + (grid_y - car_grid_y)**2
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    closest_frontier_grid = (grid_x, grid_y)
        #轉換成世界座標
        if closest_frontier_grid is not None:
            return self.grid_to_world_coordinates(closest_frontier_grid[0], closest_frontier_grid[1], map_info)
        else:
            self.ros_communicator.get_logger().info("No suitable frontier point found from random check.")
            return None

    def mark_visited_point(self, map_info, car_world_pos):
        """
        概念上會用已知的lidar/rgb/偵測環境並將範圍內的區域標記
        """
        if self.visited_map is None and map_info is not None:
            self.visited_map = np.zeros((map_info.height, map_info.width), dtype=np.int8)

        if  map_info is None or car_world_pos is None:
            self.ros_communicator.get_logger().warn("Mark visited point: Missing  map_info, or car_pose.")
            return
        
        # 轉換車輛位置到柵格座標
        car_grid_y, car_grid_x = self.world_to_grid_coordinates(
            car_world_pos[0], car_world_pos[1], map_info
        )
        # 計算可標記範圍
        sensor_range_grid = int(1.0 / map_info.resolution)  # 假設感測器範圍為1米

        # 標記車輛所在位置為已知區域 (1)
        for dy in range(-sensor_range_grid, sensor_range_grid + 1):
            for dx in range(-sensor_range_grid, sensor_range_grid + 1):
                mark_x = car_grid_x + dx
                mark_y = car_grid_y + dy
                # map.data只有0/100/-1，不須額外處理
                if (0 <= mark_x < map_info.width and 0 <= mark_y < map_info.height) :
                    
                    if self.visited_map[mark_y, mark_x] == 0:
                        self.visited_map[mark_y, mark_x] = 1
                    
    
    def exploration_logic(self):
        """
        執行探索邏輯：找到未知區域並設置為目標。返回 Nav2 的目標座標 [x, y] or None。
        """
        self.ros_communicator.get_logger().info("Attempting exploration...")
        map_data, map_info = self.data_processor.get_processed_map()
        car_pose, car_orientation = self.data_processor.get_map_basefootprint()

        if map_data is None or map_info is None or car_pose is None or car_orientation is None:
            self.ros_communicator.get_logger().warn("Exploration: Missing map, map_info, or car_pose.")
            return None
        
        # 對於車輛所在位置進行標記, 只需要car_pose的x, y座標
        car_pos_world = [car_pose[0], car_pose[1]]  
        self.mark_visited_point(map_info, car_pos_world)

        explor_target = self.find_closest_frontier_point(car_pos_world, map_data, map_info)
        if explor_target is not None:
            self.ros_communicator.get_logger().info(f"Exploration target found: {explor_target}")
            return list(explor_target)  # 返回 [x, y]
        else:
            self.ros_communicator.get_logger().info("No valid exploration target found.")
            return None
        

    #根據 Nav2 生成的路徑計算動作
    def get_action_from_nav2_plan(self, goal_coordinates=None):
        if goal_coordinates is not None and not self.goal_published_flag:
            self.ros_communicator.publish_goal_pose(goal_coordinates)
            self.goal_published_flag = True
        orientation_points, coordinates = (
            self.data_processor.get_processed_received_global_plan()
        )
        action_key = "STOP"
        if not orientation_points or not coordinates:
            action_key = "STOP"
        else:
            try:
                z, w = orientation_points[0]
                plan_yaw = get_yaw_from_quaternion(z, w)
                car_position, car_orientation = (
                    self.data_processor.get_map_basefootprint()
                )
                car_orientation_z, car_orientation_w = (
                    car_orientation[2],
                    car_orientation[3],
                )
                goal_position = self.ros_communicator.get_latest_goal()
                target_distance = cal_distance(car_position, goal_position)
                if target_distance < 0.5:
                    action_key = "STOP"
                    self.finishFlag = True
                else:
                    car_yaw = get_yaw_from_quaternion(
                        car_orientation_z, car_orientation_w
                    )
                    diff_angle = (plan_yaw - car_yaw) % 360.0
                    if diff_angle < 30.0 or (diff_angle > 330 and diff_angle < 360):
                        action_key = "FORWARD"
                    elif diff_angle > 30.0 and diff_angle < 180.0:
                        action_key = "COUNTERCLOCKWISE_ROTATION"
                    elif diff_angle > 180.0 and diff_angle < 330.0:
                        action_key = "CLOCKWISE_ROTATION"
                    else:
                        action_key = "STOP"
            except:
                action_key = "STOP"
        return action_key

    def get_action_from_nav2_plan_no_dynamic_p_2_p(self, goal_coordinates=None):
        # if goal_coordinates is not None and not self.goal_published_flag:
        #     self.ros_communicator.publish_goal_pose(goal_coordinates)
        #     self.goal_published_flag = True

        # 只抓第一次路徑
        if self.recordFlag == 0:
            if not self.check_data_availability():
                return "STOP"
            else:
                print("Get first path")
                self.index = 0
                self.global_plan_msg = (
                    self.data_processor.get_processed_received_global_plan_no_dynamic()
                )
                self.recordFlag = 1
                action_key = "STOP"

        car_position, car_orientation = self.data_processor.get_map_basefootprint()

        goal_position = self.ros_communicator.get_latest_goal()
        target_distance = cal_distance(car_position, goal_position)

        # 抓最近的物標(可調距離)
        target_x, target_y = self.get_next_target_point(car_position)

        if target_x is None or target_distance < 0.5:
            self.ros_communicator.reset_nav2()
            self.finish_nav_process()
            return "STOP"

        # 計算角度誤差
        diff_angle = self.calculate_diff_angle(
            car_position, car_orientation, target_x, target_y
        )
        if diff_angle < 20 and diff_angle > -20:
            action_key = "FORWARD"
        elif diff_angle < -20 and diff_angle > -180:
            action_key = "CLOCKWISE_ROTATION"
        elif diff_angle > 20 and diff_angle < 180:
            action_key = "COUNTERCLOCKWISE_ROTATION"
        return action_key

    #檢查是否存在初始路徑/pose/goal
    def check_data_availability(self):
        return (
            self.data_processor.get_processed_received_global_plan_no_dynamic()
            and self.data_processor.get_map_basefootprint()
            and self.ros_communicator.get_latest_goal()
        )

    def get_next_target_point(self, car_position, min_required_distance=0.5):
        """
        從global plan選擇距離車輛 min_required_distance 以上最短路徑然後返回 target_x, target_y
        """
        if self.global_plan_msg is None or self.global_plan_msg.poses is None:
            print("Error: global_plan_msg is None or poses is missing!")
            return None, None
        while self.index < len(self.global_plan_msg.poses) - 1:
            target_x = self.global_plan_msg.poses[self.index].pose.position.x
            target_y = self.global_plan_msg.poses[self.index].pose.position.y
            distance_to_target = cal_distance(car_position, (target_x, target_y))

            if distance_to_target < min_required_distance:
                self.index += 1
            else:
                self.ros_communicator.publish_selected_target_marker(
                    x=target_x, y=target_y
                    # send to /move_base_simple/goal
                )
                return target_x, target_y

        return None, None

    def calculate_diff_angle(self, car_position, car_orientation, target_x, target_y):
        target_pos = [target_x, target_y]
        diff_angle = calculate_angle_point(
            car_orientation[2], car_orientation[3], car_position[:2], target_pos
        )
        return diff_angle

    def filter_negative_onehundred(self, depth_list):
        return [depth for depth in depth_list if depth != -100.0]

    def nav2_target(self):
        target_map_pos = self.data_processor.get_processed_target_camera2map()
        if target_map_pos is None:
            self.ros_communicator.get_logger().warn("tranform camera2map failed.")
            return "STOP"

        car_pose, car_orientation = self.data_processor.get_map_basefootprint()
        if car_pose is None:
            return "STOP"
        car_pos = [car_pose[0], car_pose[1]]  # 只需要x, y座標
        car_ori = [car_orientation[2], car_orientation[3]]

        angle_diff = calculate_angle_to_target( car_pos, target_map_pos, car_ori)
        target_dist = cal_distance(car_pos, target_map_pos)

        # 設定閾值
        ANGLE_THRESHOLD = 15.0  # 度
        DISTANCE_THRESHOLD = 0.4  # 米
        self.ros_communicator.get_logger().info(
            f"Target: {target_map_pos}, Distance: {target_dist:.2f}m, Angle: {angle_diff:.2f}°"
        )

        # 根據角度和距離決定動作
        if target_dist < DISTANCE_THRESHOLD:
            self.finishFlag = True
            self.ros_communicator.get_logger().info("Reached target!")
            return "STOP"
        
        if abs(angle_diff) > ANGLE_THRESHOLD:
            # 需要轉向
            if angle_diff > 0:
                return "COUNTERCLOCKWISE_ROTATION"
            else:
                return "CLOCKWISE_ROTATION"
        else:
            # 角度OK，根據距離選擇前進速度
            if target_dist > 1.0:
                return "FORWARD"
            elif target_dist > 0.5:
                return "FORWARD_SLOW"  # 如果有慢速前進的動作
            else:
                return "FORWARD_VERY_SLOW"  # 如果有非常慢速前進的動作
        

    def camera_nav(self):
        """
        YOLO 目標資訊 (yolo_target_info) 說明：

        - 索引 0 (index 0)：
            - 表示是否成功偵測到目標
            - 0：未偵測到目標
            - 1：成功偵測到目標

        - 索引 1 (index 1)：
            - 目標的深度距離 (與相機的距離，單位為公尺)，如果沒偵測到目標就回傳 0
            - 與目標過近時(大約 40 公分以內)會回傳 -1

        - 索引 2 (index 2)：
            - 目標相對於畫面正中心的像素偏移量
            - 若目標位於畫面中心右側，數值為正
            - 若目標位於畫面中心左側，數值為負
            - 若沒有目標則回傳 0

        畫面 n 個等分點深度 (camera_multi_depth) 說明 :

        - 儲存相機畫面中央高度上 n 個等距水平點的深度值。
        - 若距離過遠、過近（小於 40 公分）或是實體相機有時候深度會出一些問題，則該點的深度值將設定為 -1。
        """
        yolo_target_info = self.data_processor.get_yolo_target_info()
        camera_multi_depth = self.data_processor.get_camera_x_multi_depth()
        yolo_target_label = self.data_processor.get_processed_yolo_target_label()
        if camera_multi_depth == None or yolo_target_info == None:
            return "STOP"

        camera_forward_depth = self.filter_negative_one(camera_multi_depth[7:13])
        camera_left_depth = self.filter_negative_one(camera_multi_depth[0:7])
        camera_right_depth = self.filter_negative_one(camera_multi_depth[13:20])

        action = "STOP"
        limit_distance = 0.7

        if all(depth > limit_distance for depth in camera_forward_depth):
            if yolo_target_info[0] == 1: 
                if yolo_target_info[2] > 200.0: 
                    action = "CLOCKWISE_ROTATION_SLOW"
                elif yolo_target_info[2] < -200.0: 
                    action = "COUNTERCLOCKWISE_ROTATION_SLOW"
                else:
                    if yolo_target_info[1] < 0.8: 
                        action = "STOP"
                    else:
                        action = "FORWARD_SLOW"
            else:
                action = "FORWARD"
        elif any(depth < limit_distance for depth in camera_left_depth):
            action = "CLOCKWISE_ROTATION"
        elif any(depth < limit_distance for depth in camera_right_depth):
            action = "COUNTERCLOCKWISE_ROTATION"
        return action

    def camera_nav_unity(self):
        """
        YOLO 目標資訊 (yolo_target_info) 說明：

        - 索引 0 (index 0)：
            - 表示是否成功偵測到目標
            - 0：未偵測到目標
            - 1：成功偵測到目標

        - 索引 1 (index 1)：
            - 目標的深度距離 (與相機的距離，單位為公尺)，如果沒偵測到目標就回傳 0
            - 與目標過近時(大約 40 公分以內)會回傳 -1

        - 索引 2 (index 2)：
            - 目標相對於畫面正中心的像素偏移量
            - 若目標位於畫面中心右側，數值為正
            - 若目標位於畫面中心左側，數值為負
            - 若沒有目標則回傳 0

        畫面 n 個等分點深度 (camera_multi_depth) 說明 :

        - 儲存相機畫面中央高度上 n 個等距水平點的深度值。
        - 若距離過遠、過近（小於 40 公分）或是實體相機有時候深度會出一些問題，則該點的深度值將設定為 -1。
        
        custom navigation logic using RRT* and PID for target approach.
        """
        lidar_data = self.data_processor.get_processed_lidar() #得到combined lidar data
        len_front_indices = 31 # lidar長度
        len_left_indices = 30  
        len_right_indices = 30
        min_lidar_dist = 0.2 # lidar min valid distance
        front_lidar_dist = lidar_data[0 : len_front_indices]
        left_lidar_dist = lidar_data[len_front_indices : len_front_indices + len_left_indices]
        right_lidar_dist = lidar_data[len_front_indices + len_left_indices : len_front_indices + len_left_indices + len_right_indices]
        # 排除小於min_lidar_dist的值
        front_lidar_dist = [d for d in front_lidar_dist if d is not None and d > min_lidar_dist and not math.isinf(d) and not math.isnan(d)]
        left_lidar_dist = [d for d in left_lidar_dist if d is not None and d > min_lidar_dist and not math.isinf(d) and not math.isnan(d)]
        right_lidar_dist = [d for d in right_lidar_dist if d is not None and d > min_lidar_dist and not math.isinf(d) and not math.isnan(d)]
        # 計算平均距離
        avg_front_distance = (
            sum(front_lidar_dist) / len(front_lidar_dist)
            if front_lidar_dist
            else float('inf')
        )
        avg_left_distance = (
            sum(left_lidar_dist) / len(left_lidar_dist)
            if left_lidar_dist
            else float('inf')
        )
        avg_right_distance = (
            sum(right_lidar_dist) / len(right_lidar_dist)
            if right_lidar_dist
            else float('inf')
        )
        #避障閾值
        obstacle_threshold = 0.3

        is_front_blocked = any(d < obstacle_threshold for d in front_lidar_dist) if front_lidar_dist else False
        is_left_blocked = any(d < obstacle_threshold for d in left_lidar_dist) if left_lidar_dist else False
        is_right_blocked = any(d < obstacle_threshold for d in right_lidar_dist) if right_lidar_dist else False


        yolo_target_label = self.data_processor.get_processed_yolo_target_label()
        yolo_target_info = self.data_processor.get_yolo_target_info()
        camera_multi_depth = self.data_processor.get_camera_x_multi_depth()
        yolo_target_info[1] *= 100.0 #換算成公分
        camera_multi_depth = list(
            map(lambda x: x * 100.0, self.data_processor.get_camera_x_multi_depth())
        )
        #預防接收不到資料
        if camera_multi_depth == None or yolo_target_info == None:
            return "STOP"
        print("camera_multi_depth:", camera_multi_depth)
        camera_forward_depth = self.filter_negative_onehundred(camera_multi_depth[6:14])
        camera_left_depth = self.filter_negative_onehundred(camera_multi_depth[0:6])
        camera_right_depth = self.filter_negative_onehundred(camera_multi_depth[14:20])
        action = "STOP"
        limit_distance = 7
        # #print("yolo_target_label:", yolo_target_label)
        # print("偵測到target?", yolo_target_info[0])
        # print("yolo_target_info[1]:", yolo_target_info[1])
        # #print("depth(camera_forward_depth):", camera_forward_depth)
        # if is_front_blocked is False:
        #     if yolo_target_info[0] == 1:  # 成功偵測到目標
        #         print("目標偏左(<-200)偏右(>200)", yolo_target_info[2])
        #         if yolo_target_info[2] > 200.0: # 目標偏右
        #             action = "CLOCKWISE_ROTATION_SLOW"
        #         elif yolo_target_info[2] < -200.0: # 目標偏左
        #             action = "COUNTERCLOCKWISE_ROTATION_SLOW"
        #         else:
        #             if yolo_target_info[1] < limit_distance: # 深度距離小於 6
        #                 self.finishFlag = True
        #                 action = "STOP"
        #             else:
        #                 action = "FORWARD"
        #     else: # yolo_target_info[0] == 0
        #             #選擇更空曠的地方前進
        #             action = "FORWARD"   
        # elif any(depth < limit_distance for depth in camera_right_depth) or is_right_blocked:
        #     action = "COUNTERCLOCKWISE_ROTATION"
        # elif any(depth < limit_distance for depth in camera_left_depth) or is_left_blocked:
        #     print("depth:", camera_left_depth)
        #     action = "CLOCKWISE_ROTATION"
        
        # return action

        # 1. YOLO目標已非常接近，準備停止
        if (yolo_target_info[0] == 1 and (yolo_target_info[1] < limit_distance) and (abs(yolo_target_info[2]) < 100) ): # 假設已轉為cm
            self.finishFlag = True
            action = "STOP"
            self.ros_communicator.get_logger().info("Target very close, stopping.")
            return action
        

        # 2. 近距離避障 (深度相機& LiDAR)
        is_cam_front_near_blocked = any(d_cm < limit_distance for d_cm in camera_forward_depth) if camera_forward_depth else False
        is_cam_left_near_blocked = any(d_cm < limit_distance for d_cm in camera_left_depth) if camera_left_depth else False
        is_cam_right_near_blocked = any(d_cm < limit_distance for d_cm in camera_right_depth) if camera_right_depth else False

        if is_cam_front_near_blocked:
            self.ros_communicator.get_logger().info("CameraDepth: Front NEARLY blocked.")
            if not is_cam_left_near_blocked and not is_left_blocked: # lidar and camera 左邊皆為安全距離
                action = "COUNTERCLOCKWISE_ROTATION"
            elif not is_cam_right_near_blocked and not is_right_blocked: 
                action = "CLOCKWISE_ROTATION"
            else:
                action = "BACKWARD_SLOW" # 近處前方堵死
            self.ros_communicator.get_logger().info(f"Near Obstacle Action: {action}")
            return action

        # 3. 中距離避障 (only LiDAR)
        if is_front_blocked:
            self.ros_communicator.get_logger().info("LiDAR: Front blocked.")
            if not is_right_blocked: action = "CLOCKWISE_ROTATION"
            elif not is_left_blocked: action = "COUNTERCLOCKWISE_ROTATION"
            else: action = "BACKWARD_SLOW" # LiDAR 前左右都堵
            self.ros_communicator.get_logger().info(f"LiDAR Front Obstacle Action: {action}")
            return action
        elif is_left_blocked: # LiDAR 左方有障礙
            self.ros_communicator.get_logger().info("LiDAR: Left blocked.")
            action = "CLOCKWISE_ROTATION" # 向右轉
            self.ros_communicator.get_logger().info(f"LiDAR Left Obstacle Action: {action}")
            return action
        elif is_right_blocked: # LiDAR 右方有障礙
            self.ros_communicator.get_logger().info("LiDAR: Right blocked.")
            action = "COUNTERCLOCKWISE_ROTATION" # 向左轉
            self.ros_communicator.get_logger().info(f"LiDAR Right Obstacle Action: {action}")
            return action

        # 4. 沒有近距離和中距離障礙，執行目標追蹤或探索
        self.ros_communicator.get_logger().debug("No immediate obstacles detected by LiDAR or near CameraDepth.")
        if yolo_target_info[0] == 1:  # 偵測到目標
            if yolo_target_info[2] > 150.0:
                action = "CLOCKWISE_ROTATION_SLOW"
            elif yolo_target_info[2] < -150.0:
                action = "COUNTERCLOCKWISE_ROTATION_SLOW"
            else: 
                action = "FORWARD" 
            self.ros_communicator.get_logger().info(f"Target Tracking Action: {action}")
        else: # 未偵測到目標，執行沿牆探索
            min_wall_dist = obstacle_threshold + 0.5 # 比避障閾值稍遠一點，比如0.5m
            max_wall_dist = min_wall_dist + 0.7 
            
            # avg_right_distance 是公尺
            if avg_right_distance < min_wall_dist : # 離右牆太近
                action = "COUNTERCLOCKWISE_ROTATION_SLOW"
                self.ros_communicator.get_logger().info(f"Wall Following: Too close to right wall ({avg_right_distance:.2f}m), turning left slightly.")
            elif avg_right_distance > max_wall_dist and avg_right_distance != float('inf'): # 離右牆太遠 (且右邊有牆)
                action = "CLOCKWISE_ROTATION_SLOW"
                self.ros_communicator.get_logger().info(f"Wall Following: Too far from right wall ({avg_right_distance:.2f}m), turning right slightly.")
            elif avg_right_distance == float('inf'): 
                # 可以選擇繼續直行，或者嘗試尋找牆（例如，小角度右轉）
                action = "FORWARD" # 或者 "CLOCKWISE_ROTATION_VERY_SLOW"
                self.ros_communicator.get_logger().info("Wall Following: No right wall detected or too far, moving forward.")
            else: 
                action = "FORWARD"
                self.ros_communicator.get_logger().info(f"Wall Following: Good distance to right wall ({avg_right_distance:.2f}m), moving forward.")
        return action

    def stop_nav(self):
        return "STOP"


'''
/odom 在底盤啟動時建立，根據車輛速度和位置的更新與世界座標系偏差
/amcl_pose 是 AMCL (Adaptive Monte Carlo Localization) 的輸出，
提供車輛在地圖上的位置和方向，AMCL得到的结果是map->odom的TF，即對
odometry定位的修正。
'''