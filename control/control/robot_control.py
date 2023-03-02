import numpy as np
from std_msgs.msg import Int16MultiArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_srvs.srv import Empty
import time

from interbotix_xs_msgs.msg import JointGroupCommand,JointSingleCommand,JointTrajectoryCommand

class Robot_control(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.robot1_ns = 'arm_1'
        self.robot2_ns = 'arm_2'

        gripper_pressure_lower_limit = 150
        gripper_pressure = 0.5
        gripper_pressure_upper_limit = 350
        self.gripper_value = gripper_pressure_lower_limit + (gripper_pressure
        * (gripper_pressure_upper_limit - gripper_pressure_lower_limit)
        )

        self.move_time  = 2.0

        gripper_name = 'gripper'
        self.gripper_command = JointSingleCommand(name=gripper_name)


        # self.timer = self.create_timer(1/10, self.timer_callback)
        self.robot1_joints_pub = self.create_publisher(
            JointGroupCommand, f'{self.robot1_ns}/commands/joint_group', 10
        )
        self.robot2_joints_pub = self.create_publisher(
            JointGroupCommand, f'{self.robot2_ns}/commands/joint_group', 10
        )
        self.robot1_single_pub = self.create_publisher(
            JointSingleCommand, f'{self.robot1_ns}/commands/joint_single', 10
        )
        self.robot2_single_pub = self.create_publisher(
            JointSingleCommand, f'{self.robot2_ns}/commands/joint_single', 10
        )

        self.robot1_home_srv = self.create_service(Empty, 'home1', self.home1_callback)
        self.robot1_sleep_srv = self.create_service(Empty, 'sleep1', self.sleep1_callback)
        self.robot1_ft_srv = self.create_service(Empty, 'ft1', self.ft1_callback)
        self.robot1_ht_srv = self.create_service(Empty, 'ht1', self.ht1_callback)
        self.robot1_idle1_srv = self.create_service(Empty, 'idle1', self.idle1_callback)

        self.robot1_grasp_srv = self.create_service(Empty, 'grasp1', self.grasp1_callback)
        self.robot1_release_srv = self.create_service(Empty, 'release1', self.release1_callback)

        self.robot2_home_srv = self.create_service(Empty, 'home2', self.home2_callback)
        self.robot2_sleep_srv = self.create_service(Empty, 'sleep2', self.sleep2_callback)
        self.robot2_ft_srv = self.create_service(Empty, 'fb2', self.fb2_callback)
        self.robot2_ht_srv = self.create_service(Empty, 'hb2', self.hb2_callback)
        self.robot2_idle_srv = self.create_service(Empty, 'idle2', self.idle2_callback)

        self.robot2_grasp_srv = self.create_service(Empty, 'grasp2', self.grasp2_callback)
        self.robot2_release_srv = self.create_service(Empty, 'release2', self.release2_callback)
    # def timer_callback(self):
    #     # if (self.vel_group.size >10):
    #     #     self.vel_group.



    def home1_callback(self,request, response):
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [0.0,0.0,0.0,0.0]
        self.robot1_joints_pub.publish(msg)
        time.sleep(self.move_time)
        return Empty.Response()
    
    def sleep1_callback(self,request, response):
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [0.0, -1.88, 1.5, 0.8]
        self.robot1_joints_pub.publish(msg)
        time.sleep(self.move_time)
        return Empty.Response()
    
    def home2_callback(self,request, response):
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [0.0,0.0,0.0,0.0]
        self.robot2_joints_pub.publish(msg)
        time.sleep(self.move_time)
        return Empty.Response()
    
    def sleep2_callback(self,request, response):
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [0.0, -1.88, 1.5, 0.8]
        self.robot2_joints_pub.publish(msg)
        time.sleep(self.move_time)
        return Empty.Response()
#full throttle

    def grasp_r1(self):
        self.gripper_command.cmd = -self.gripper_value
        self.robot1_single_pub.publish(self.gripper_command)
        time.sleep(1.0)

    def grasp_r2(self):
        self.gripper_command.cmd = -self.gripper_value
        self.robot2_single_pub.publish(self.gripper_command)
        time.sleep(1.0)

    def release_r1(self):
        self.gripper_command.cmd = self.gripper_value
        self.robot1_single_pub.publish(self.gripper_command)
        time.sleep(1.0)

    def release_r2(self):
        self.gripper_command.cmd = self.gripper_value
        self.robot2_single_pub.publish(self.gripper_command)
        time.sleep(1.0)


#gripper callbacks

    def grasp1_callback(self,request, response):
        self.grasp_r1()
        return Empty.Response()

    def release1_callback(self,request, response):
        self.release_r1()
        return Empty.Response()
    
    def grasp2_callback(self,request, response):
        self.grasp_r2()
        return Empty.Response()

    def release2_callback(self,request, response):
        self.release_r2()
        return Empty.Response()

    def ft1_callback(self,request, response):
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [0.0, -1.88, 1.5, 0.8]
        self.robot2_joints_pub.publish(msg)

        time.sleep(self.move_time)


        return Empty.Response()
#half throttle
    def ht1_callback(self,request, response):
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [0.0, -1.88, 1.5, 0.8]
        self.robot2_joints_pub.publish(msg)
        return Empty.Response()
#throttle idle
    def idle1_callback(self,request, response):
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [0.0, -1.88, 1.5, 0.8]
        self.robot2_joints_pub.publish(msg)
        return Empty.Response()
#full brake
    def fb2_callback(self,request, response):
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [0.0, -1.88, 1.5, 0.8]
        self.robot2_joints_pub.publish(msg)
        return Empty.Response()
#half brake
    def hb2_callback(self,request, response):
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [0.0, -1.88, 1.5, 0.8]
        self.robot2_joints_pub.publish(msg)
        return Empty.Response()
#brake idle
    def idle2_callback(self,request, response):
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [0.0, -1.88, 1.5, 0.8]
        self.robot2_joints_pub.publish(msg)
        return Empty.Response()



def main(args=None):
    rclpy.init(args=args)
    control = Robot_control()
    rclpy.spin(control)
    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()