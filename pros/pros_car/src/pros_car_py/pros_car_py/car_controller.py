from rclpy.node import Node
from std_msgs.msg import String
from pros_car_py.car_models import DeviceDataTypeEnum, CarCControl
import threading
import time


class CarController:

    def __init__(self, ros_communicator, nav_processing):
        self.ros_communicator = ros_communicator
        self.nav_processing = nav_processing
        # 用來管理後台執行緒的屬性
        self._auto_nav_thread = None
        self._stop_event = None
        self._thread_running = False
        self.flag = 0

        self._auto_nav_thread = None
        self._stop_event = threading.Event()
        self._thread_running = False

        self.target_idx = 0  # 目標索引
        self.target_list = [
            [0.12577216615733916, 4.207528556910003],
            [0.004709751367064641, -0.43933601070552486],
            [3.202388878639925, 3.893176401328583],
        ]

    def update_action(self, action_key):
        """
        Updates the velocity for each of the car's wheels.

        Args:
            vel1 (float): Velocity for the rear left wheel (rad/s).
            vel2 (float): Velocity for the rear right wheel (rad/s).
            vel3 (float): Velocity for the front left wheel (rad/s).
            vel4 (float): Velocity for the front right wheel (rad/s).

        Example:
            car_controller.update_velocity(10, 10, 10, 10)  # Set all wheels' velocity to 10 rad/s.
        """
        self.ros_communicator.publish_car_control(action_key)

    def manual_control(self, key):
        """
        Controls the car based on single character inputs ('w', 'a', 's', 'd', 'z').

        Args:
            key (str): A single character representing a control command.
                'w' - move forward
                's' - move backward
                'a' - turn left
                'd' - turn right
                'z' - stop

        Example:
            car_controller.manual_control('w')  # Moves the car forward.
        """
        if key == "w":
            self.update_action("FORWARD")
        elif key == "s":
            self.update_action("BACKWARD")
        elif key == "a":
            self.update_action("LEFT_FRONT")
        elif key == "d":
            self.update_action("RIGHT_FRONT")
        elif key == "e":
            self.update_action("COUNTERCLOCKWISE_ROTATION")
        elif key == "r":
            self.update_action("CLOCKWISE_ROTATION")
        elif key == "z":
            self.update_action("STOP")
        elif key == "q":
            self.update_action("STOP")
            time.sleep(0.1)
            return True

        else:
            pass

    def auto_control(self, mode="manual_auto_nav", target=None, key=None):
        """
        自動控制邏輯
        Args:
            mode: 控制模式 ("auto_nav" 或 "manual_nav")
            target: 目標座標 (用於 manual_nav 模式)
            key: 鍵盤輸入
        """
        # 如果有按鍵輸入
        if self.flag == 0:
            stop_event = threading.Event()
            thread = threading.Thread(target=self.background_task, args=(stop_event,))

        if key == "q":
            # 按下 q 時停止導航並退出
            if self._thread_running:
                self._stop_event.set()
                self._auto_nav_thread.join()
                self._thread_running = False

            self.nav_processing.reset_nav_process()
            action_key = "STOP"
            self.ros_communicator.publish_car_control(
                action_key, publish_rear=True, publish_front=True
            )
            return True

        if not self._thread_running:
            self._stop_event.clear()  # 清除之前的停止狀態
            self._auto_nav_thread = threading.Thread(
                target=self.background_task,
                args=(self._stop_event, mode, target),
                daemon=True,
            )
            self._auto_nav_thread.start()
            self._thread_running = True

        return False

    def stop_nav(self):
        for i in range(20):
            time.sleep(0.1)
            self.update_action("STOP")

    def background_task(self, stop_event, mode, target):
        """
        後台任務：不斷執行導航動作直到 stop_event 被設定。
        """

        while not stop_event.is_set():

            if mode == "manual_auto_nav":
                action_key = (
                    self.nav_processing.get_action_from_nav2_plan_no_dynamic_p_2_p(
                        goal_coordinates=None
                    )
                )
                if self.nav_processing.get_finish_flag():
                    self.nav_processing.reset_nav_process()
            elif mode == "target_auto_nav":

                current_target = self.target_list[self.target_idx]
                # 把goal_coordinates傳到get_action_from_nav2_plan_no_dynamic_p_2_p
                action_key = (
                    self.nav_processing.get_action_from_nav2_plan_no_dynamic_p_2_p(
                        goal_coordinates=current_target
                    )
                )
                if self.nav_processing.get_finish_flag():
                    self.nav_processing.reset_nav_process()
                    self.target_idx = (self.target_idx + 1) % len(self.target_list)
                    continue
            
            # 發布控制指令, choose custom_nav 會觸發nav_processing的camera_nav_unity()                        
            elif mode == "custom_nav":
                action_key = "STOP"
                self.ros_communicator.publish_init_pose()
                if self.nav_processing.current_explor_target is None or self.nav_processing.get_finish_flag():
                    if self.nav_processing.get_finish_flag():
                        self.nav_processing.reset_nav_process()
                        self.ros_communicator.reset_nav2()
                        self.nav_processing.current_explor_target = None
                    explor_target = self.nav_processing.exploration_logic()
                    if explor_target is not None:
                        self.nav_processing.current_explor_target = explor_target
                        self.ros_communicator.publish_goal_pose(self.nav_processing.current_explor_target)
                        self.nav_processing.goal_published_flag = True
                        self.ros_communicator.get_logger().info(f"Custom_nav: New exploration goal published: {self.nav_processing.current_explor_target}")
                        action_key = "STOP"
                        received_plan = False
                        for count in range(5):
                            time.sleep(0.5)
                            temp_plan = self.nav_processing.data_processor.get_processed_received_global_plan_no_dynamic()
                            if temp_plan is not None:
                                received_plan = True
                                self.ros_communicator.get_logger().info(f"Custom_nav: Received global plan.")
                                time.sleep(0.5)  # 等待導航計算完成
                                break
                            else:
                                self.ros_communicator.get_logger().warn(f"Waiting for path... attempt {count+1}/10")
                        if not received_plan:
                            self.ros_communicator.get_logger().error("Failed to receive path after 10 attempts, resetting target")
                            self.nav_processing.current_explor_target = None
                            self.nav_processing.goal_published_flag = False
                    else:
                        action_key = "STOP"
                        self.ros_communicator.get_logger().info("No exploration target found.")
                # 如果有當前探索目標且未完成導航
                if self.nav_processing.current_explor_target is not None and not self.nav_processing.get_finish_flag():
                    self.ros_communicator.get_logger().debug(f"Custom_nav: Following path to {self.nav_processing.current_explor_target}")
                    action_key = self.nav_processing.get_action_from_nav2_plan_no_dynamic_p_2_p(
                        goal_coordinates=self.nav_processing.current_explor_target # 傳遞當前目標
                    )
                    # get_finish_flag 會在 get_action_from_nav2_plan_no_dynamic_p_2_p 內部被更新
                    if self.nav_processing.get_finish_flag():
                        self.ros_communicator.get_logger().info(f"Custom_nav: Reached exploration target {self.nav_processing.current_explor_target} or path failed.")
                        self.nav_processing.current_explor_target = None # 清除，以便下次重新探索
                        # reset_nav_process 已經在上面 if 條件為真時調用過了
                elif self.nav_processing.current_explor_target is None and not self.nav_processing.get_finish_flag():
                # 這種情況通常是 exploration_logic 沒找到目標
                    action_key = "STOP"

                # 獨立的邏輯檢查是否偵測到Pikachu
                yolo_info = self.nav_processing.data_processor.get_yolo_target_info()
                target_label = self.nav_processing.data_processor.get_processed_yolo_target_label()
                if yolo_info and yolo_info[0] == 1 and target_label == "Pikachu": # find Pikachu
                    self.ros_communicator.get_logger().info("PIKACHU DETECTED! Overriding exploration.")
                    self.ros_communicator.clear_received_global_plan() 
                    #原本要使用/yolo/detection/position的資料，但是無法取得frame_id的座標系轉換
                    action_key_pikachu = self.nav_processing.nav2_target()
                    #action_key_pikachu = self.nav_processing.camera_nav_unity()
                    if self.nav_processing.get_finish_flag(): # 如果 camera_nav_unity 說完成了 (到達Pikachu)
                        self.ros_communicator.get_logger().info("Successfully reached Pikachu!")
                        action_key = "STOP"
                        self.nav_processing.current_explor_target = None
                        self.nav_processing.reset_nav_process() # 重置導航處理狀態
                        self.ros_communicator.reset_nav2()
                        stop_event.set() # 停止後台任務
                    else:
                        action_key = action_key_pikachu # 如果還沒完成，就繼續使用 camera_nav_unity 的動作
                if self._thread_running == False:
                    action_key = "STOP"
                print(action_key)
                time.sleep(0.1)  # 等待一段時間以確保控制指令被處理
                self.ros_communicator.publish_car_control(
                    action_key, publish_rear=True, publish_front=True
                )
        
        # 收尾動作
        print("[background_task] Navigation stopped.")

    def run(self, mode, target):
        pass
