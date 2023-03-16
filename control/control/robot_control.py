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
        self.robot1_ns = 'arm_2'
        self.robot2_ns = 'arm_1'

        gripper_pressure_lower_limit = 150
        gripper_pressure = 0.5
        gripper_pressure_upper_limit = 350
        self.gripper_value = gripper_pressure_lower_limit + (gripper_pressure
        * (gripper_pressure_upper_limit - gripper_pressure_lower_limit)
        )

        self.move_time  = 2.0

        gripper_name = 'gripper'
        self.gripper_command = JointSingleCommand(name=gripper_name)

        self.gripper1_command = JointSingleCommand(name=gripper_name)
        self.gripper2_command = JointSingleCommand(name=gripper_name)

        self.T1 = [0.01, 0.35, -0.77, 0.1]   #IT
        self.T2 = [0.03,-0.60, 0.60, 0.0]    #FT
        self.T3 = [0.025, -0.25, 0.15, 0.0]   #HT
        self.B1 = [0.04,-0.85, 0.95, 0.0]   #IB1
        self.B2 = [0.03,0.0, 0.07, 0.0]   #FB
        self.B4 = [0.04,-0.95, 0.99, 0.0]   #IB2
        self.B3 = [0.04,-0.35, 0.4, 0.0]    #HB
        self.S = [0.0, -1.88, 1.5, 0.8]     #sleep

        self.msg1 = JointGroupCommand()
        self.msg1.name = 'arm'

        self.msg2 = JointGroupCommand()
        self.msg2.name = 'arm'


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

        self.FT_srv = self.create_service(Empty, 'FT', self.FT_callback)
        self.HT_srv = self.create_service(Empty, 'HT', self.HT_callback)
        self.ID_srv = self.create_service(Empty, 'ID', self.ID_callback)
        self.HB_srv = self.create_service(Empty, 'HB', self.HB_callback)
        self.FB_srv = self.create_service(Empty, 'FB', self.FB_callback)

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

        self.robot_sleep_srv = self.create_service(Empty, 'sleep', self.sleep_callback)
    # def timer_callback(self):
    #     # if (self.vel_group.size >10):
    #     #     self.vel_group.

        self.cmd_sub = self.create_subscription(
            Int16,
            'command',
            self.cmd_callback,
            1)
        
        self.prev_cmd = 2
        self.current_cmd = 2
        
    def cmd_callback(self, msg):
        # self.current_cmd = int(msg.data)
        self.function_test(int(msg.data))

    def function_test(self,msg):
        self.current_cmd = int(msg)
        if self.current_cmd == 0:
            if self.prev_cmd ==2:
                #TODO: T1/grasp/T2  None
                self.msg1.cmd = self.T1
                self.robot1_joints_pub.publish(self.msg1)
                time.sleep(self.move_time)

                self.grasp_r1()
                self.msg1.cmd = self.T2
                self.robot1_joints_pub.publish(self.msg1)

                time.sleep(self.move_time)


            elif self.prev_cmd ==1:
                #TODO: T2   None
                self.msg1.cmd = self.T2
                self.robot1_joints_pub.publish(self.msg1)
                time.sleep(self.move_time)
            elif self.prev_cmd ==3 or self.prev_cmd == 4:
                #TODO: T1/grasp/T2  B4/release/s
                self.msg1.cmd = self.T1
                self.msg2.cmd = self.B4
                self.robot1_joints_pub.publish(self.msg1)
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)

                self.gripper1_command.cmd = -self.gripper_value #Close
                self.gripper2_command.cmd = self.gripper_value  #Open
                self.robot1_single_pub.publish(self.gripper1_command)
                self.robot2_single_pub.publish(self.gripper2_command)
                time.sleep(2.5)

                self.msg1.cmd = self.T2
                self.msg2.cmd = self.S
                self.robot1_joints_pub.publish(self.msg1)
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)


        elif self.current_cmd == 1:
            if self.prev_cmd ==2:
                #TODO: T1/grasp/T3	None
                self.msg1.cmd = self.T1
                self.robot1_joints_pub.publish(self.msg1)
                time.sleep(self.move_time)

                self.grasp_r1()
                self.msg1.cmd = self.T3
                self.robot1_joints_pub.publish(self.msg1)

                time.sleep(self.move_time)

            elif self.prev_cmd ==0:
                #TODO: T3	None
                self.msg1.cmd = self.T3
                self.robot1_joints_pub.publish(self.msg1)
                time.sleep(self.move_time)
            elif self.prev_cmd ==3 or self.prev_cmd == 4:
                #TODO: T1/grasp/T3	B4/release/s
                self.msg1.cmd = self.T1
                self.msg2.cmd = self.B4
                self.robot1_joints_pub.publish(self.msg1)
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)

                self.gripper1_command.cmd = -self.gripper_value #Close
                self.gripper2_command.cmd = self.gripper_value  #Open
                self.robot1_single_pub.publish(self.gripper1_command)
                self.robot2_single_pub.publish(self.gripper2_command)
                time.sleep(2.5)

                self.msg1.cmd = self.T3
                self.msg2.cmd = self.S
                self.robot1_joints_pub.publish(self.msg1)
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)

        elif self.current_cmd == 2:
            if self.prev_cmd ==0 or self.prev_cmd ==1:
                #TODO: T1/release/s	None
                self.msg1.cmd = self.T1
                self.robot1_joints_pub.publish(self.msg1)
                time.sleep(self.move_time)
                self.gripper1_command.cmd = self.gripper_value #Open
                self.robot1_single_pub.publish(self.gripper1_command)
                time.sleep(2.5)
                self.msg1.cmd = self.S
                self.robot1_joints_pub.publish(self.msg1)
                time.sleep(self.move_time)


            elif self.prev_cmd ==3 or self.prev_cmd == 4:
                #TODO: None	B4/release/s
                self.msg2.cmd = self.B4
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)
                self.gripper2_command.cmd = self.gripper_value #Open
                self.robot2_single_pub.publish(self.gripper2_command)
                time.sleep(2.5)
                self.msg2.cmd = self.S
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)

        elif self.current_cmd == 3:
            if self.prev_cmd ==0 or self.prev_cmd ==1:
                #TODO: T1/release/s	B1/grasp/B3
                self.msg1.cmd = self.T1
                self.msg2.cmd = self.B1
                self.robot1_joints_pub.publish(self.msg1)
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)

                self.gripper1_command.cmd = self.gripper_value  #Open
                self.gripper2_command.cmd = -self.gripper_value #Close
                self.robot1_single_pub.publish(self.gripper1_command)
                self.robot2_single_pub.publish(self.gripper2_command)
                time.sleep(2.5)

                self.msg1.cmd = self.S
                self.msg2.cmd = self.B3
                self.robot1_joints_pub.publish(self.msg1)
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)
            elif self.prev_cmd ==2:
                #TODO: None	B1/grasp/B3
                self.msg2.cmd = self.B1
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)

                self.grasp_r2()
                self.msg2.cmd = self.B3
                self.robot2_joints_pub.publish(self.msg2)

                time.sleep(self.move_time)
            elif self.prev_cmd == 4:
                #TODO: None	B3
                self.msg2.cmd = self.B3
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)

        elif self.current_cmd == 4:
            if self.prev_cmd ==0 or self.prev_cmd ==1:
                #TODO: T1/release/s	B1/grasp/B2
                self.msg1.cmd = self.T1
                self.msg2.cmd = self.B1
                self.robot1_joints_pub.publish(self.msg1)
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)

                self.gripper1_command.cmd = self.gripper_value  #Open
                self.gripper2_command.cmd = -self.gripper_value #Close
                self.robot1_single_pub.publish(self.gripper1_command)
                self.robot2_single_pub.publish(self.gripper2_command)
                time.sleep(2.5)

                self.msg1.cmd = self.S
                self.msg2.cmd = self.B2
                self.robot1_joints_pub.publish(self.msg1)
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)
            elif self.prev_cmd ==2:
                #TODO: None	B1/grasp/B2
                self.msg2.cmd = self.B1
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)

                self.grasp_r2()
                self.msg2.cmd = self.B2
                self.robot2_joints_pub.publish(self.msg2)

                time.sleep(self.move_time)
            elif self.prev_cmd == 3:
                #TODO: None	B2
                self.msg2.cmd = self.B2
                self.robot2_joints_pub.publish(self.msg2)
                time.sleep(self.move_time)


        self.prev_cmd = self.current_cmd

    def FT_callback(self,request, response):
        self.function_test(0)
        return Empty.Response()
    
    def HT_callback(self,request, response):
        self.function_test(1)
        return Empty.Response()

    def ID_callback(self,request, response):
        self.function_test(2)
        return Empty.Response()

    def HB_callback(self,request, response):
        self.function_test(3)
        return Empty.Response()

    def FB_callback(self,request, response):
        self.function_test(4)
        return Empty.Response()

    def sleep_callback(self,request, response):
        self.current_cmd = 2
        self.prev_cmd = 2
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [0.0, -1.88, 1.5, 0.8]
        self.robot1_joints_pub.publish(msg)
        self.robot2_joints_pub.publish(msg)
        time.sleep(self.move_time)


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
        time.sleep(2.0)

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