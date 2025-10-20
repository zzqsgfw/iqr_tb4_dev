#!/usr/bin/env python3
import struct
import time
import math
import threading
from collections import deque
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
dev = '/dev/input/js1' # nomachine will occupy the jso

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

class DirectJoyToCmdVel(Node):
    def __init__(self):
        super().__init__('direct_joy_to_cmd_vel')
        
        # 创建cmd_vel发布者
        self.publisher = self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 初始化手柄读取器
        self.joystick = JoystickReader(dev)
        
        # 控制参数
        self.linear_scale = 0.5
        self.angular_scale = 1.0
        self.max_linear_speed = 2.0
        self.max_angular_speed = 3.0
        
        # 启动手柄读取
        if not self.joystick.start():
            self.get_logger().error("无法启动手柄读取器，节点将退出")
            return
        
        # 创建定时器发布cmd_vel
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)  # 20Hz
        
        self.get_logger().info('直接手柄控制节点已启动')
        
        # 打印手柄映射信息
        self.get_logger().info('手柄映射:')
        self.get_logger().info('左拨杆上下: 轴1, 左拨杆左右: 轴0')
        self.get_logger().info('右拨杆上下: 轴4, 右拨杆左右: 轴3')
        self.get_logger().info('左上肩键: 按钮4, 右上肩键: 按钮5')
        self.get_logger().info('左下肩键: 按钮6, 右下肩键: 按钮7')
    
    def get_control_mode(self):
        """根据肩键状态返回控制模式"""
        buttons = self.joystick.buttons
        
        # 模式1: 左摇杆控制线速度，右摇杆控制角速度
        if buttons[4] == 0 and buttons[5] == 0:  # 无肩键按下
            return "normal"
        # 模式2: 左肩键按下时减速
        elif buttons[4] == 1:
            return "slow"
        # 模式3: 右肩键按下时加速
        elif buttons[5] == 1:
            return "fast"
        # 模式4: 左下肩键急停
        elif buttons[6] == 1:
            return "stop"
        
        return "normal"
    
    def publish_cmd_vel(self):
        """发布cmd_vel消息"""
        if not self.joystick.running:
            return
            
        twist = Twist()
        
        try:
            # 获取控制模式
            mode = self.get_control_mode()
            
            if mode == "stop":
                # 急停模式：发布零速度
                self.publisher.publish(twist)
                return
            
            # 获取摇杆值
            left_y = self.joystick.axes[1]  # 左拨杆上下 (前进/后退)
            left_x = self.joystick.axes[0]  # 左拨杆左右 (左右平移)
            right_x = self.joystick.axes[3] # 右拨杆左右 (旋转)
            
            # 基础速度计算
            linear_scale = self.linear_scale
            angular_scale = self.angular_scale
            
            # 根据模式调整速度
            if mode == "slow":
                linear_scale *= 0.3
                angular_scale *= 0.3
            elif mode == "fast":
                linear_scale *= 1.5
                angular_scale *= 1.5
            
            # 设置速度
            # 前进/后退 (左拨杆上下，注意Y轴方向)
            twist.linear.x = -left_y * linear_scale * self.max_linear_speed
            # 左右平移 (左拨杆左右)
            twist.linear.y = left_x * linear_scale * self.max_linear_speed
            # 旋转 (右拨杆左右)
            twist.angular.z = -right_x * angular_scale * self.max_angular_speed
            
            # 发布速度命令
            self.publisher.publish(twist)
            
            # 调试信息（限制频率避免输出过多）
            if hasattr(self, '_last_log_time') is False:
                self._last_log_time = time.time()
            
            current_time = time.time()
            if current_time - self._last_log_time > 1.0:  # 每秒输出一次
                if any([abs(twist.linear.x) > 0.01, abs(twist.linear.y) > 0.01, abs(twist.angular.z) > 0.01]):
                    mode_text = {
                        "normal": "正常",
                        "slow": "慢速",
                        "fast": "快速",
                        "stop": "急停"
                    }
                    self.get_logger().info(
                        f'模式: {mode_text[mode]} | '
                        f'线速度: x={twist.linear.x:.2f}, y={twist.linear.y:.2f} | '
                        f'角速度: z={twist.angular.z:.2f}'
                    )
                self._last_log_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'处理手柄数据时出错: {e}')
    
    def destroy_node(self):
        """重写销毁方法，确保正确清理"""
        self.joystick.stop()
        # 发布停止命令
        stop_twist = Twist()
        self.publisher.publish(stop_tswist)
        self.get_logger().info('发布停止命令并关闭节点')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DirectJoyToCmdVel()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行错误: {e}")
    finally:
        # 确保节点正确销毁
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()