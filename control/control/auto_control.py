import numpy as np
from std_msgs.msg import Int16MultiArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_srvs.srv import Empty
from enum import Enum, auto
import pygame
# import time

class Train_State(Enum):
    """Current state of the system."""

    """Determines what the main timer function should be doing on each iteration."""
    FT = auto(),
    HT = auto(),
    IDLE = auto(),
    HB = auto(),
    FB = auto(),
    MAN = auto(),

class Station_State(Enum):
    """Current state of the system."""

    """Determines what the main timer function should be doing on each iteration."""
    APP = auto(),
    NAN = auto(),

class Limit_State(Enum):
    """Current state of the system."""

    """Determines what the main timer function should be doing on each iteration."""
    NAN = auto(),
    ACC = auto(),

class Crossing_State(Enum):
    """Current state of the system."""

    """Determines what the main timer function should be doing on each iteration."""
    APP = auto(),
    NAN = auto(),

class Auto_control(Node):
    def __init__(self):
        super().__init__('auto_control')
        self.timer = self.create_timer(1/10, self.timer_callback)
        self.speed_sub = self.create_subscription(
            Int16,
            'speed',
            self.speed_callback,
            10)
        self.sign_sub = self.create_subscription(
            Int16MultiArray,
            'signs',
            self.sign_callback,
            10)

        self.resume_srv = self.create_service(Empty, 'resume', self.resume_callback)

        self.command_pub = self.create_publisher(Int16, 'command', 10)

        self.vel_now = 0.0                              # current vel
        self.acc = 0.0                                  # current acc
        self.vel_group = np.zeros(20,dtype=float)       # recording speed info
        self.vel_group_last = np.zeros(20,dtype=float)
        self.vel_check = 0.0                            # velocity after 20s
        self.dis = 0.0                                  # distance recorder
        self.speed_limit = 10                           # current speed limit
        self.speed_curve = 80.0                         # expected speed for station approaching
        self.station_dis = 1.0

        self.next_limit = 10.0                           # next speed limit
        self.temp_limit_dis = 1.0                       # calculate distance to predict limit change
        self.train_lenght = 0.0510985         # train length 5 sharyo bilevel and one f40ph in miles
        self.signs = []                                 # detected signs
        self.command = 'N'
        self.corss_state = Crossing_State.NAN  
        self.station_state = Station_State.NAN
        self.limit_state = Limit_State.NAN
        self.train_state = Train_State.IDLE

        self.control_cmd = 2

        self.speed_thres = 2.0

        self.prev_cmd = self.control_cmd

        self.cmd = Int16()

        ## GUI Stuff
        pygame.init()
        self.win=pygame.display.set_mode((400, 400))
        pygame.display.set_caption("Train Info")
        self.Font1=pygame.font.SysFont('freesansbold.ttf',  60)
        self.Font2=pygame.font.SysFont('freesansbold.ttf',  40)
        self.white=(255, 255, 255)
        self.black=(0, 0, 0)
        self.Speed_GUI=self.Font1.render("Speed: "+ str(self.vel_now) +"MPH", True, self.black)
        self.Control_GUI=self.Font1.render("Control: "+ str(self.command), False, self.black)
        self.Distance_GUI=self.Font2.render("Odometer: "+ str(self.dis)+"Miles", False, self.black)
        self.Station_GUI=self.Font2.render("Next Station: "+ str(self.station_dis)+"Miles", False, self.black)
        self.Limit_GUI=self.Font2.render("Speed Limit: "+ str(self.speed_limit)+"MPH", False, self.black)


    def speed_callback(self, msg):
        self.vel_now = float(msg.data)*0.1

    def timer_callback(self):
        # if (self.vel_group.size >10):
        #     self.vel_group.
        self.vel_group_last = self.vel_group
        self.vel_group = np.roll(self.vel_group,-1)
        self.vel_group[-1] = self.vel_now
        self.acc = (np.sum(self.vel_group-self.vel_group_last))*360*5 # miles pre sec^2
        self.vel_check = self.vel_now+self.acc/3600*6
        self.dis = self.dis + self.vel_now/36000
        # self.get_logger().info(f'dis {self.dis:.2f} speed {self.vel_now:.2f} acc {self.acc:.2f} nextspeed6s {self.vel_check:.2f}')

        self.state_update()
        self.threshold_update()

        if self.speed_limit <= self.speed_curve:
            limit = self.speed_limit
        else:
            limit = self.speed_curve

        if self.vel_now > limit:
            if self.vel_now<limit+2 and self.station_state == Station_State.NAN:
                self.train_state = Train_State.HB
                self.control_cmd = 3
            else:
                self.train_state = Train_State.FB
                self.control_cmd = 4
        else:
            if self.vel_now<limit and self.vel_now>limit-self.speed_thres and self.station_state == Station_State.NAN:
                self.train_state = Train_State.IDLE
                self.control_cmd = 2
            elif self.vel_now<limit-self.speed_thresa and self.station_state == Station_State.NAN:
                if self.vel_now<=10.5 and limit < 15:
                    self.train_state = Train_State.HT
                    self.control_cmd = 1
                else:
                    self.train_state = Train_State.FT
                    self.control_cmd = 0
            else:
                self.train_state = Train_State.IDLE
                self.control_cmd = 0

        # if self.vel_now <= self.speed_limit-2 and self.vel_now <= self.speed_curve-3 and self.vel_check <= self.speed_limit-3:
        #     self.train_state = Train_State.ACC
        #     self.get_logger().info(f'D {self.vel_now:.2f}')
        # elif self.vel_now <= self.speed_limit and self.vel_now <= self.speed_curve:
        #     self.train_state = Train_State.IDLE
        #     self.get_logger().info(f'N {self.vel_now:.2f}')
        # elif self.vel_now > self.speed_limit or self.vel_now > self.speed_curve:
        #     self.train_state = Train_State.DEC
        #     self.get_logger().info(f'B {self.vel_now:.2f}')
        # elif self.speed_curve == 10.0:
        #     self.train_state = Train_State.MAN
        #     self.get_logger().info(f'HUMAN speed:{self.vel_now:.2f} station end:{self.station_dis}')
        if self.control_cmd != self.prev_cmd:
            self.cmd.data = int(self.control_cmd)
            self.command_pub.publish(self.cmd)

        self.prev_cmd = self.control_cmd

        self.control_update()

    def threshold_update(self):
        if self.speed_limit<=15.0:
            self.speed_thres = 2.0
        else:
            self.speed_thres = 2-(self.vel_now-15.0)/100.0

    def state_update(self):
        if self.limit_state == Limit_State.ACC:
            self.temp_limit_dis += self.vel_now/36000
            self.get_logger().info(f'limit passed {self.temp_limit_dis:.6f}, passed limit: {self.next_limit:.2f}')
            if self.temp_limit_dis>=self.train_lenght:
                self.limit_state = Limit_State.NAN
                self.speed_limit = self.next_limit
        if self.station_state == Station_State.APP:
            self.station_dis -= self.vel_now/36000
            if self.station_dis >= 0.1:
                self.speed_curve = self.station_dis*90
            else:
                self.speed_curve = 10.0

    def control_update(self):
        if self.train_state == Train_State.FT:
            self.command = 'FT'
        elif self.train_state == Train_State.HT:
            self.command = 'HT'
        elif self.train_state == Train_State.IDLE:
            self.command = 'N'
        elif self.train_state == Train_State.HB:
            self.command = 'HB'
        elif self.train_state == Train_State.FB:
            self.command = 'FB'
        else:
            self.command = 'M'

        self.win.fill(self.white)

        self.Speed_GUI=self.Font1.render("Speed: "+ str(self.vel_now) +"MPH", True, self.black)
        self.Control_GUI=self.Font1.render("Control: "+ str(self.command), False, self.black)
        self.Distance_GUI=self.Font2.render("Odometer: "+ str(self.dis)+"Miles", False, self.black)
        self.Station_GUI=self.Font2.render("Next Station: "+ str(self.station_dis)+"Miles", False, self.black)
        self.Limit_GUI=self.Font1.render("Speed Limit: "+ str(self.speed_limit)+"MPH", False, self.black)
        self.win.blit(self.Speed_GUI, (10, 10))
        self.win.blit(self.Limit_GUI, (10, 70))
        self.win.blit(self.Control_GUI, (10, 130))
        self.win.blit(self.Distance_GUI, (10, 180))
        self.win.blit(self.Station_GUI, (10, 230))
        pygame.display.update()


    def resume_callback(self,request, response):
        self.station_state = Station_State.NAN
        self.speed_curve = 80
        self.station_dis = 1.0
        return Empty.Response()


    def sign_callback(self, msg):
        self.signs = msg.data
        for i in range(len(self.signs)):
            if self.signs[i] == 0 or self.signs[i] == 1:
                self.next_limit = 79
            elif self.signs[i] == 2 or self.signs[i] == 3:
                self.next_limit = 40
            elif self.signs[i] == 4:
                self.next_limit = 75
            elif self.signs[i] == 5 or self.signs[i] == 12:
                self.next_limit = 25
            elif self.signs[i] == 6:
                self.next_limit = 20
            elif self.signs[i] == 7:
                self.next_limit = 65
            elif self.signs[i] == 8:
                self.corss_state = Crossing_State.APP
            elif self.signs[i] == 9:
                self.station_state = Station_State.APP
                self.station_dis = 1.0
            # elif self.signs[i] == 10:
            # elif self.signs[i] == 11:
            elif self.signs[i] == 13:
                self.next_limit = 70

        if self.next_limit < self.speed_limit-1:
            self.speed_limit = self.next_limit
            self.limit_state = Limit_State.NAN
        elif self.next_limit > self.speed_limit and self.limit_state == Limit_State.NAN:
            self.limit_state = Limit_State.ACC
            self.temp_limit_dis = -0.0

        self.get_logger().info(f'next limit {self.next_limit:.2f}, limit: {self.speed_limit:.2f}')



def main(args=None):
    print("1")
    rclpy.init(args=args)
    control = Auto_control()
    rclpy.spin(control)
    control.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()