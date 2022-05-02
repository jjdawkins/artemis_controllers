import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import math
import numpy as np
import quaternion

def saturation(val,min,max):

    if(val > max):
        val = max

    if(val < min):
        val = min

    return val

class DroneController(Node):

    def __init__(self):
        super().__init__('drone_contoller')
        self.publisher_ = self.create_publisher(OverrideRCIn, '/rc/override/raw', 10)
        self.ctrl_dt = 0.02  # seconds

        self.set_roll = 0.0
        self.set_pitch = 0.0
        self.set_yaw_rate = 0.0
        self.set_thrust = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.throttle = 0.0
        self.max_xy_vel = 1.0 # m/s
        self.max_z_vel = 1.0        
        self.max_rp_ang = 15*math.pi/180 # 15 Degree Angle
        self.max_yaw_rate = 1.57 # Rad/s

        
        self.vel_x_cmd = 0.0
        self.vel_y_cmd = 0.0
        self.vel_z_cmd = 0.0
        self.yaw_rate_cmd = 0.0
        self.Kvel = 0.3 # 0.17 gives 10 Degree tilt for 1.0 m/s of error
        self.Katt = 1.0 #0.63 # gives 
        self.Kz = 0.3
        self.Kzi = 0.05
        self.z_err_int = 0
        self.z_err = 0

        self.arm = False
        self.pos = np.zeros([3,1])
        self.vel_g = np.zeros([3,1])
        self.vel_b = np.zeros([3,1])
        self.quat = np.quaternion(1,0,0,0)
        self.eul = np.zeros([3,1]) # yaw pitch roll
        
        self.timer = self.create_timer(self.ctrl_dt, self.control_loop)
        self.i = 0

        self.odom_sub = self.create_subscription(Odometry,'/drone/mocap/odom',self.odom_callback,10)
        self.cmd_vel_sub = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.joy_sub = self.create_subscription(Joy,'/joy',self.joy_callback,10)
        self.joy_sub  # prevent unused variable warning


    def odom_callback(self,msg):
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = msg.pose.pose.position.z

        self.quat = np.quaternion(msg.pose.pose.orientation.w,
                                  msg.pose.pose.orientation.x,
                                  msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z)

        self.vel_b[0] = msg.twist.twist.linear.x
        self.vel_b[1] = msg.twist.twist.linear.y
        self.vel_b[2] = msg.twist.twist.linear.z

        R = quaternion.as_rotation_matrix(self.quat)

        self.eul[0] = math.atan2(R[1][0],R[0][0])
        self.eul[1] = -math.asin(R[2][0])
        self.eul[2] = math.atan2(R[2][1],R[2][2]) 
        #print(self.yaw,self.pitch,self.roll)

    def cmd_vel_callback(self,msg):
        self.vel_x_cmd = msg.twist.linear.x
        self.vel_y_cmd = msg.twist.linear.y
        self.vel_z_cmd = msg.twist.linear.z
        self.yaw_rate_cmd = msg.twist.angular.z

    def joy_callback(self,msg):
        if(msg.buttons[5]):
            self.arm = True

        if(msg.buttons[4]):
            self.arm = False

        self.vel_x_cmd = self.max_xy_vel*msg.axes[4]
        self.vel_y_cmd = self.max_xy_vel*msg.axes[3]
        self.vel_z_cmd = self.max_z_vel*msg.axes[1]
        self.yaw_rate_cmd = self.max_yaw_rate*msg.axes[0]


        #self.roll = (msg.axes[3] * -500) + 1500
        #self.pitch = (msg.axes[4] * 500) + 1500
        #self.yaw = (msg.axes[0] * -500) + 1500
        self.throttle = (msg.axes[1] * 1000) + 1000

        if(self.throttle < 1000):
            self.throttle = 1000

        #print(self.roll, self.pitch, self.yaw, self.throttle)


    def control_loop(self):
        
        # Set Desired Roll and Pitch Angle based on Desired B-frame Velocity
        self.roll_set = -self.Kvel*(self.vel_y_cmd - self.vel_b[1])
        self.pitch_set = self.Kvel*(self.vel_x_cmd - self.vel_b[0])        
        
        # Set Desired Roll and Pitch Rate based on Desired Angles
        roll_rate_set = self.Katt*(self.roll_set - self.eul[2])
        pitch_rate_set = self.Katt*(self.pitch_set - self.eul[1])
        
        roll_rate_set = saturation(roll_rate_set,-1.0,1.0)
        pitch_rate_set = saturation(pitch_rate_set,-1.0,1.0)
        yaw_rate_set = saturation(self.yaw_rate_cmd/self.max_yaw_rate,-1.0,1.0)
        
       # print(roll_rate_set ,pitch_rate_set,yaw_rate_set)

        self.z_err = self.vel_z_cmd - self.vel_b[2]
        self.z_err_int = self.z_err_int + self.z_err*self.ctrl_dt
        self.z_err_int = saturation(self.z_err_int,-0.5,0.5)
        
        thr_set = self.Kz*(self.z_err) + self.Kzi*self.z_err_int + 0.4  
              
        thr_set = saturation(thr_set,0.0,1.0)

       # self.roll = (roll_rate_set * -500) + 1500
       # self.pitch = (pitch_rate_set * 500) + 1500
       # self.yaw = (msg.axes[0] * -500) + 1500
        #self.throttle = (self.thr_set * 1000) + 1000

        #if(self.throttle < 1000):
        #    self.throttle = 1000

        msg = OverrideRCIn()
        if(self.arm):        
            msg.channels[0] = (roll_rate_set * -500) + 1500
            msg.channels[1] = (pitch_rate_set * 500) + 1500
            msg.channels[2] = (thr_set * 1000) + 1000
           # msg.channels[2] = self.throttle
            msg.channels[3] = (yaw_rate_set * -500) + 1500        
            msg.channels[4] = 2000
        else:
            msg.channels[0] = 1500
            msg.channels[1] = 1500
            msg.channels[2] = 1000
            msg.channels[3] = 1500             
            msg.channels[4] = 1000

        self.publisher_.publish(msg)
        #print(msg.channels)
        #self.i += 1


def main(args=None):
    rclpy.init(args=args)

    my_drone = DroneController()

    rclpy.spin(my_drone)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
