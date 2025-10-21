#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import subprocess
import threading
import time
import os
import math
import struct
import select

# 摇杆设备路径
dev = '/dev/input/js0'

class JoystickReader:
    def __init__(self, device=dev):
        self.device = device
        self.file = None
        self.running = False
        self.thread = None
        
        # 手柄状态存储
        self.axes = [0.0] * 8
        self.buttons = [0] * 11
        self.axis_values = {}  # 存储原始值
        
        # 控制参数
        self.deadzone = 5000
        self.joystick_max = 32767
        self.joystick_min = -32767
        
    def normalize_joystick(self, value):
        """归一化拨杆值到[-1, 1]范围"""
        if abs(value) < self.deadzone:
            return 0.0
        
        # 归一化到[-1, 1]
        normalized = value / self.joystick_max
        
        # 应用死区补偿
        if normalized > 0:
            normalized = (normalized - self.deadzone/self.joystick_max) / (1 - self.deadzone/self.joystick_max)
        else:
            normalized = (normalized + self.deadzone/self.joystick_max) / (1 - self.deadzone/self.joystick_max)
        
        return max(-1.0, min(1.0, normalized))
    
    def open_device(self):
        """打开手柄设备"""
        try:
            self.file = open(self.device, 'rb')
            print(f"成功打开手柄设备: {self.device}")
            return True
        except Exception as e:
            print(f"无法打开手柄设备 {self.device}: {e}")
            return False
    
    def read_joystick_event(self):
        """读取手柄事件"""
        try:
            event = self.file.read(8)
            if len(event) == 8:
                return struct.unpack('IhBB', event)
            return None
        except (OSError, struct.error) as e:
            print(f"读取手柄事件错误: {e}")
            return None
    
    def update_state(self, event):
        """更新手柄状态"""
        timestamp, value, event_type, number = event
        
        if event_type == 0x02:  # 摇杆事件
            if number < len(self.axes):
                self.axis_values[number] = value
                self.axes[number] = self.normalize_joystick(value)
                
        elif event_type == 0x01:  # 按钮事件
            if number < len(self.buttons):
                self.buttons[number] = value
    
    def start(self):
        """开始读取手柄数据"""
        if not self.open_device():
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True
        self.thread.start()
        return True
    
    def _read_loop(self):
        """手柄数据读取循环"""
        print("开始读取手柄数据...")
        while self.running:
            try:
                # 使用select检查是否有数据可读
                r, w, e = select.select([self.file], [], [], 0.1)
                if r:
                    event = self.read_joystick_event()
                    if event:
                        self.update_state(event)
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"手柄读取循环错误: {e}")
                time.sleep(0.1)
    
    def stop(self):
        """停止读取"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.file:
            self.file.close()

class PX100ForwardKinematics:
    """PX100机械臂正向运动学结算"""
    
    def __init__(self):
        # PX100机械臂DH参数 (单位: 米)
        self.dh_params = [
            # a, alpha, d, theta_offset
            [0.0, 0.0, 0.093, 0.0],           # waist -> shoulder
            [0.0, math.pi/2, 0.0, math.pi/2], # shoulder -> elbow  
            [0.135, 0.0, 0.0, 0.0],           # elbow -> wrist
            [0.147, 0.0, 0.0, 0.0]            # wrist -> end effector
        ]
        
    def dh_matrix(self, a, alpha, d, theta):
        """计算DH变换矩阵"""
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        
        return [
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ]
    
    def forward_kinematics(self, joint_angles):
        """
        正向运动学计算
        joint_angles: [waist, shoulder, elbow, wrist_angle] 弧度
        """
        if len(joint_angles) != 4:
            raise ValueError("需要4个关节角度")
        
        # 初始变换矩阵 (基坐标系)
        T = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
        
        # 计算每个关节的变换矩阵
        for i in range(4):
            a, alpha, d, theta_offset = self.dh_params[i]
            theta = joint_angles[i] + theta_offset
            T_i = self.dh_matrix(a, alpha, d, theta)
            
            # 矩阵乘法
            T_new = [[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]]
            for i in range(4):
                for j in range(4):
                    for k in range(4):
                        T_new[i][j] += T[i][k] * T_i[k][j]
            T = T_new
        
        return T
    
    def get_position(self, joint_angles):
        """获取末端执行器位置"""
        T = self.forward_kinematics(joint_angles)
        position = [T[0][3], T[1][3], T[2][3]]
        return position

class ArmController(Node):
    def __init__(self):
        super().__init__("ArmController")
        
        # 机械臂控制
        self.arm_group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.arm_timer_pub = self.create_timer(0.1, self.timer_cb)
        self.joint_states_sub = self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 10)
        
        # 摇杆控制
        self.joystick = JoystickReader(dev)
        if not self.joystick.start():
            self.get_logger().error("无法启动手柄读取器")
        
        # 机械臂控制参数
        self.arm_group_command = JointGroupCommand()
        self.arm_group_command.name = "arm"
        self.arm_group_command.cmd = [0.0, -0.3, 0.8, 1.0]  # 初始位置
        
        # 运动学计算
        self.fk = PX100ForwardKinematics()
        
        # 状态变量
        self._joint_pos = []
        self._cnt = 0
        self.control_mode = "base"  # base: 底盘控制, arm: 机械臂控制
        
        # 控制参数
        self.arm_speed = -0.5  # 机械臂控制速度
        self.base_linear_scale = 0.5
        self.base_angular_scale = 1.0
        
        self.get_logger().info("机械臂控制器已初始化")

    def joint_states_cb(self, msg):
        if len(msg.name) == 7:
            self._joint_pos.clear()
            for i in range(4):
                self._joint_pos.append(msg.position[i])

            if self._cnt % 20 == 0:  # 减少日志频率
                # 计算运动学
                if len(self._joint_pos) == 4:
                    position = self.fk.get_position(self._joint_pos)
                    self.get_logger().info(f"\n关节状态 - waist: {msg.position[0]:.3f}, shoulder: {msg.position[1]:.3f}, elbow: {msg.position[2]:.3f}, wrist: {msg.position[3]:.3f}")
                    self.get_logger().info(f"末端位置 - X: {position[0]:.3f}m, Y: {position[1]:.3f}m, Z: {position[2]:.3f}m")

    def get_control_mode(self):
        """根据肩键状态返回控制模式"""
        buttons = self.joystick.buttons
        
        # 模式切换: 右上肩键切换底盘/机械臂控制
        if buttons[5] == 1:  # 右上肩键
            return "arm"
        else:
            return "base"

    def control_base(self):
        """控制底盘移动"""
        twist = Twist()
        
        # 获取摇杆值
        left_y = self.joystick.axes[1]  # 左拨杆上下 (前进/后退)
        left_x = self.joystick.axes[0]  # 左拨杆左右 (左右平移)
        right_x = self.joystick.axes[3] # 右拨杆左右 (旋转)
        
        # 设置速度
        twist.linear.x = -left_y * self.base_linear_scale
        twist.linear.y = left_x * self.base_linear_scale
        twist.angular.z = -right_x * self.base_angular_scale
        
        self.cmd_vel_pub.publish(twist)
        
        # 输出控制信息
        if self._cnt % 10 == 0:
            if any([abs(twist.linear.x) > 0.01, abs(twist.linear.y) > 0.01, abs(twist.angular.z) > 0.01]):
                self.get_logger().info(f'底盘控制 - 线速度: x={twist.linear.x:.2f}, y={twist.linear.y:.2f} | 角速度: z={twist.angular.z:.2f}')

    def control_arm(self):
        """控制机械臂"""
        if len(self._joint_pos) != 4:
            return
            
        # 获取摇杆值
        left_x = self.joystick.axes[0]  # 左拨杆左右 -> 控制X坐标 (waist关节)
        left_y = self.joystick.axes[1]  # 左拨杆上下 -> 控制Y坐标 
        right_y = self.joystick.axes[4] # 右拨杆上下 -> 控制Z坐标
        
        # 更新关节角度
        new_cmd = list(self.arm_group_command.cmd)
        
        # 左摇杆控制waist和shoulder关节 (XY平面)
        if abs(left_x) > 0.1:
            new_cmd[0] += left_x * self.arm_speed * 0.1  # waist
        if abs(left_y) > 0.1:
            new_cmd[1] += -left_y * self.arm_speed * 0.1  # shoulder
            
        # 右摇杆控制elbow关节 (Z方向)
        if abs(right_y) > 0.1:
            new_cmd[2] += -right_y * self.arm_speed * 0.1  # elbow
        
        # 限制关节范围
        new_cmd[0] = max(-3.14, min(3.14, new_cmd[0]))    # waist
        new_cmd[1] = max(-1.57, min(1.57, new_cmd[1]))    # shoulder
        new_cmd[2] = max(0.0, min(2.0, new_cmd[2]))       # elbow
        new_cmd[3] = max(-1.57, min(1.57, new_cmd[3]))    # wrist
        
        self.arm_group_command.cmd = new_cmd
        
        # 输出控制信息
        if self._cnt % 10 == 0:
            position = self.fk.get_position(new_cmd)
            self.get_logger().info(f'机械臂控制 - 关节: {[f"{angle:.2f}" for angle in new_cmd]}')
            self.get_logger().info(f'末端位置 - X: {position[0]:.3f}m, Y: {position[1]:.3f}m, Z: {position[2]:.3f}m')

    def timer_cb(self):
        """主控制循环"""
        self._cnt += 1
        
        # 检查控制模式
        new_mode = self.get_control_mode()
        if new_mode != self.control_mode:
            self.control_mode = new_mode
            self.get_logger().info(f"切换到 {self.control_mode} 控制模式")
        
        # 根据模式执行控制
        if self.control_mode == "base":
            self.control_base()
        else:  # arm mode
            self.control_arm()
            self.arm_group_pub.publish(self.arm_group_command)

    def destroy_node(self):
        """清理资源"""
        self.joystick.stop()
        # 发布停止命令
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        self.get_logger().info('发布停止命令并关闭节点')
        super().destroy_node()

def launch_iqr_tb4_bringup():
    """启动iqr_tb4 bringup"""
    print("启动iqr_tb4 bringup...")
    
    iqr_tb4_bringup_dir = "/home/tony/ros2_ws/src/iqr_tb4_ros/iqr_tb4_bringup"
    
    bringup_launch_file = os.path.join(iqr_tb4_bringup_dir, "launch", "bringup.launch.py")
    
    if not os.path.exists(bringup_launch_file):
        print(f"错误: 找不到bringup启动文件 {bringup_launch_file}")
        return None
    
    try:
        # 启动bringup进程
        process = subprocess.Popen(
            ['ros2', 'launch', f'{iqr_tb4_bringup_dir}/launch/bringup.launch.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # 等待bringup初始化
        print("等待iqr_tb4 bringup初始化...")
        time.sleep(8)
        
        return process
    except Exception as e:
        print(f"启动iqr_tb4 bringup错误: {e}")
        return None

def main():
    # 首先启动iqr_tb4 bringup
    bringup_process = launch_iqr_tb4_bringup()
    
    if bringup_process is None:
        print("无法启动iqr_tb4 bringup，退出")
        return
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建并运行控制器
        print("启动机械臂控制器...")
        controller = ArmController()
        
        # 运行控制器
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("正在关闭...")
    except Exception as e:
        print(f"执行错误: {e}")
    finally:
        # 清理
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()
        
        # 终止bringup进程
        if bringup_process:
            print("终止iqr_tb4 bringup...")
            bringup_process.terminate()
            bringup_process.wait()

if __name__ == '__main__':
    main()