#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from sensor_msgs.msg import JointState
import subprocess
import threading
import time
import os

class ArmController(Node):
    def __init__(self):
        super().__init__("ArmController")
        self.arm_group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        self.arm_timer_pub = self.create_timer(0.1, self.timer_cb)
        self.joint_states_sub = self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 10)
        
        self.arm_group_command = JointGroupCommand()
        self.arm_group_command.name = "arm"
        self.arm_group_command.cmd = [0.0, -0.3, 0.8, 1.0]

        self._joint_pos = []
        self._cnt = 0
        self.get_logger().info("ArmController initialized")

    def joint_states_cb(self, msg):
        if len(msg.name) == 7:
            self._joint_pos.clear()
            for i in range(4):
                self._joint_pos.append(msg.position[i])

            if self._cnt % 20 == 0:  # Reduce logging frequency
                self.get_logger().info("\nReceived joint states:\n"
                    f"- waist: {msg.position[0]:.3f}\n"
                    f"- shoulder: {msg.position[1]:.3f}\n"
                    f"- elbow: {msg.position[2]:.3f}\n"
                    f"- wrist_angle: {msg.position[3]:.3f}"
                )

    def timer_cb(self):
        self.arm_group_pub.publish(self.arm_group_command)
        self._cnt += 1

        if self._cnt % 10 == 0:
            if len(self._joint_pos) == 4:
                print(self._joint_pos)

def launch_iqr_tb4_bringup():
    """Launch iqr_tb4 bringup in a separate process"""
    print("Launching iqr_tb4 bringup...")
    
    # You'll need to adjust this path to your actual iqr_tb4_bringup directory
    iqr_tb4_bringup_dir = "/home/tony/ros2_ws/src/iqr_tb4_ros/iqr_tb4_bringup"  # CHANGE THIS PATH
    
    bringup_launch_file = os.path.join(iqr_tb4_bringup_dir, "launch", "bringup.launch.py")
    
    if not os.path.exists(bringup_launch_file):
        print(f"Error: Could not find bringup launch file at {bringup_launch_file}")
        print("Please update the iqr_tb4_bringup_dir path in the script")
        return None
    
    try:
        # Launch the bringup process
        process = subprocess.Popen(
            ['ros2', 'launch', f'{iqr_tb4_bringup_dir}/launch/bringup.launch.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Wait a bit for the bringup to initialize
        print("Waiting for iqr_tb4 bringup to initialize...")
        time.sleep(5)
        
        return process
    except Exception as e:
        print(f"Error launching iqr_tb4 bringup: {e}")
        return None

def main():
    # First, launch the iqr_tb4 bringup
    bringup_process = launch_iqr_tb4_bringup()
    
    if bringup_process is None:
        print("Failed to launch iqr_tb4 bringup. Exiting.")
        return
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and run the ArmController
        print("Starting ArmController...")
        controller = ArmController()
        
        # Run the controller
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error during execution: {e}")
    finally:
        # Cleanup
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()
        
        # Terminate the bringup process
        if bringup_process:
            print("Terminating iqr_tb4 bringup...")
            bringup_process.terminate()
            bringup_process.wait()

if __name__ == '__main__':
    main()