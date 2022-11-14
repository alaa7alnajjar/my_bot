#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


from dynamixel_sdk import *                    


#####################################################################################################################################################################



class dxl_robot(Node):

    def __init__(self):
        super().__init__("motor_control")
        self.sub_speed = 0
        self.sub_angle = 0
        
        self.subscriber_speed = self.create_subscription(String, "move_data_speed", self.callback_move_data_speed, 10)
        self.subscriber_angle = self.create_subscription(String, "move_data_angle", self.callback_move_data_angle, 10)
        self.get_logger().info("movement data subscriber has been started")

        self.robot_init()
        


    def robot_init(self):

        # Control table address
        self.ADDR_OPERATING_MODE         = 11               # Control table addresses 
        self.ADDR_TORQUE_ENABLE          = 64               
        self.ADDR_GOAL_VELOCITY          = 104
        self.ADDR_PRESENT_VELOCITY       = 128

        # Protocol version
        PROTOCOL_VERSION                 = 2.0               

        # Default setting
        self.DXL_ID1                      = 1                 # Dynamixel ID : 1
        self.DXL_ID2                      = 2                 # Dynamixel ID : 2

        BAUDRATE                    = 1000000                # Dynamixel default baudrate : 57600
        DEVICENAME                  = '/dev/ttyACM0'         # dxl driver board port  >> Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        VELOCITY_CONTROL_MODE            = 1

        
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

        # Set operating mode
        dxl1_comm_result, dxl1_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID1, self.ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
        dxl2_comm_result, dxl2_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID2, self.ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)

        if dxl1_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl1_comm_result))
        elif dxl1_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl1_error))
        else:
            print("Operating mode changed to Velocity Mode")

        if dxl2_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl2_comm_result))
        elif dxl2_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl2_error))
        else:
            print("Operating mode changed to Velocity Mode")

        # Enable Dynamixel Torque
        dxl1_comm_result, dxl1_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID1, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl1_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl1_comm_result))
        elif dxl1_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl1_error))
        else:
            print("Dynamixel 1 has been successfully connected")

        dxl2_comm_result, dxl2_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID2, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl2_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl2_comm_result))
        elif dxl2_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl2_error))
        else:
            print("Dynamixel 2 has been successfully connected")



    def move_robot(self, L_velocity, R_velocity):
        
        if (L_velocity > 200): L_velocity = 200
        if (L_velocity < -200): L_velocity = -200
        if (R_velocity > 200): R_velocity = 200
        if (R_velocity < -200): R_velocity = -200
        # Write goal position    
        dxl1_comm_result, dxl1_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID1, self.ADDR_GOAL_VELOCITY, L_velocity)         
        if dxl1_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl1_comm_result))
        elif dxl1_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl1_error))

        dxl2_comm_result, dxl2_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID2, self.ADDR_GOAL_VELOCITY, R_velocity)          
        if dxl2_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl2_comm_result))
        elif dxl2_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl2_error))



    def input_movement(self, required_speed = 0, required_angle = 0):


        while (required_angle > 90): required_angle = int(90)
        while (required_angle < -90): required_angle = int(-90)
        while (required_speed > 200): required_speed = 200
        while (required_speed < -200): required_speed = -200

        
        L_speed = required_speed + required_angle                               #dxl1
        R_speed = required_speed - required_angle                               #dxl2
        self.move_robot(L_speed,R_speed)


    def torque_off(self):
        self.move_robot(0,0)

        # Disable Dynamixel Torque
        dxl1_comm_result, dxl1_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID1, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl1_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl1_comm_result))
        elif dxl1_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl1_error))

        dxl2_comm_result, dxl2_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID2, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl2_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl2_comm_result))
        elif dxl2_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl2_error))
        # Close port

        self.portHandler.closePort()


    def callback_move_data_speed(self, speed_input):
        #self.get_logger().info(speed_input.data)

        self.sub_speed = speed_input.data
        self.input_movement(int(self.sub_speed), int(self.sub_angle))


    def callback_move_data_angle(self, angle_input):
        #self.get_logger().info(angle_input.data)

        self.sub_angle = angle_input.data
        self.input_movement(int(self.sub_speed), int(self.sub_angle))




    
############################################################################################################################################################



def main(args=None):
    rclpy.init(args=args)

    robot = dxl_robot()

    #speed_input = int(input("enter speed: "))
    #angle_input = int(input("enter degree: "))

    
    rclpy.spin(robot)

    robot.torque_off()            #######
    rclpy.shutdown()

    





if __name__ == "__main__":

    main()















#########################################################################################################################################################################
# Notes/Todo




'''

while 1:
   

    # Write goal position    
    dxl1_comm_result, dxl1_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_VELOCITY, 20)          ####################
    if dxl1_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl1_comm_result))
    elif dxl1_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl1_error))

    dxl2_comm_result, dxl2_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_VELOCITY, -200)          ####################
    if dxl2_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl2_comm_result))
    elif dxl2_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl2_error))

    # Read present position
    dxl1_present_velocity, dxl1_comm_result, dxl1_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID1, ADDR_PRESENT_VELOCITY)
    if dxl1_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl1_comm_result))
    elif dxl1_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl1_error))

    print("   [ID:%03d] GoalVelocity:%03d  PresVelocity:%03d" %(DXL_ID1, dxl1_goal_velocity, dxl1_present_velocity), end = "\r") 
    
'''




