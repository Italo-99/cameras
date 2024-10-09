#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

class RoverStatusMonitor:
    
    def __init__(self):
        rospy.init_node('rover_status_monitor', anonymous=True)

        # Expected sizes for different message types
        self.expected_sizes = {
            '/sirio/car/motors/pos_status'      : 8,
            '/sirio/car/motors/vel_status'      : 8,
            '/sirio/arm/motors/pos_status'      : 7,
            '/sirio/arm/motors/vel_status'      : 7,
            '/sirio/systems/motors/pos_status'  : 4,
            '/sirio/systems/motors/vel_status'  : 4,
            '/sirio/energy/battery'             : 5,
            '/sirio/energy/current_sensors'     : 4,
            '/sirio/systems/loadcell_1'         : 2,
            '/sirio/systems/loadcell_2'         : 2,
            '/sirio/systems/loadcell_3'         : 2,
        }

        # Car Arm and systems motors' position and velocity status
        rospy.Subscriber('/sirio/car/motors/pos_status',     Float32MultiArray, self.car_pos_callback)
        rospy.Subscriber('/sirio/car/motors/vel_status',     Float32MultiArray, self.car_vel_callback)
        rospy.Subscriber('/sirio/arm/motors/pos_status',     Float32MultiArray, self.arm_pos_callback)
        rospy.Subscriber('/sirio/arm/motors/vel_status',     Float32MultiArray, self.arm_vel_callback)
        rospy.Subscriber('/sirio/systems/motors/pos_status', Float32MultiArray, self.tls_pos_callback)
        rospy.Subscriber('/sirio/systems/motors/vel_status', Float32MultiArray, self.tls_vel_callback)

        # Energy status
        rospy.Subscriber('/sirio/energy/battery',         Float32MultiArray, self.battery_callback)
        rospy.Subscriber('/sirio/energy/current_sensors', Float32MultiArray, self.current_sensors_callback)
        
        # Loadcells
        rospy.Subscriber('/sirio/systems/loadcell_1', Float32MultiArray, self.loadcell_1_callback)
        rospy.Subscriber('/sirio/systems/loadcell_2', Float32MultiArray, self.loadcell_2_callback)
        rospy.Subscriber('/sirio/systems/loadcell_3', Float32MultiArray, self.loadcell_3_callback)        
        
        # Sensors offset computation
        # self.counter_sensing    = 0
        # self.sensor_1           = 0
        # self.sensor_2           = 0
        # self.sensor_3           = 0
        # self.sensor_4           = 0

    def check_size(self, topic_name, data_size):
        expected_size = self.expected_sizes.get(topic_name, None)
        if expected_size is not None and data_size != expected_size:
            rospy.logwarn(f"Received data size {data_size} on topic {topic_name} does not match expected size {expected_size}.")
            return False
        elif expected_size is None:
            rospy.logwarn(f"Received data size {data_size} on topic {topic_name} with variable size.")
            return False
        return True

    def car_pos_callback(self, data):   
        if (self.check_size('/sirio/car/motors/pos_status', len(data.data))):
            rospy.loginfo(f"Car Motor Positions, Wheels FR: {data.data[0]:.2f}, RR: {data.data[1]:.2f}, RL: {data.data[2]:.2f}, FL: {data.data[3]:.2f}")
            rospy.loginfo(f"Car Motor Positions, Steers FR: {data.data[4]:.2f}, RR: {data.data[5]:.2f}, RL: {data.data[6]:.2f}, FL: {data.data[7]:.2f}\n")

    def car_vel_callback(self, data):
        if (self.check_size('/sirio/car/motors/vel_status', len(data.data))):
            rospy.loginfo(f"Car Motor Velocities, Wheels FR: {data.data[0]:.2f}, RR: {data.data[1]:.2f}, RL: {data.data[2]:.2f}, FL: {data.data[3]:.2f}")
            rospy.loginfo(f"Car Motor Velocities, Steers FR: {data.data[4]:.2f}, RR: {data.data[5]:.2f}, RL: {data.data[6]:.2f}, FL: {data.data[7]:.2f}\n")

    def arm_pos_callback(self, data):
        if (self.check_size('/sirio/arm/motors/pos_status', len(data.data))):
            rospy.loginfo(f"Arm Motor Positions, Joint1: {data.data[0]:.2f}, Joint2: {data.data[1]:.2f}, Joint3: {data.data[2]}, Joint4: {data.data[3]:.2f}")
            rospy.loginfo(f"Arm Motor Positions, Joint5: {data.data[4]:.2f}, Joint6: {data.data[5]:.2f}, EE: {data.data[6]:.2f}\n")

    def arm_vel_callback(self, data):
        if (self.check_size('/sirio/arm/motors/vel_status', len(data.data))):
            rospy.loginfo(f"Arm Motor Velocities, Joint1: {data.data[0]:.2f}, Joint2: {data.data[1]:.2f}, Joint3: {data.data[2]:.2f}, Joint4: {data.data[3]:.2f}")
            rospy.loginfo(f"Arm Motor Velocities, Joint5: {data.data[4]:.2f}, Joint6: {data.data[5]:.2f}, EE: {data.data[6]:.2f}\n")

    def tls_pos_callback(self, data):
        if (self.check_size('/sirio/systems/motors/pos_status', len(data.data))):
            rospy.loginfo(f"Tools system Motor Positions, Drill Tip: {data.data[0]:.2f}, Drill Mover: {data.data[1]:.2f}, Sand box motor: {data.data[2]}, Probes container motor: {data.data[3]:.2f}\n")

    def tls_vel_callback(self, data):
        if (self.check_size('/sirio/systems/motors/vel_status', len(data.data))):
            rospy.loginfo(f"Tools system Motor Velocities, Drill Tip: {data.data[0]:.2f}, Drill Mover: {data.data[1]:.2f}, Sand box motor: {data.data[2]:.2f}, Probes container motor: {data.data[3]:.2f}\n")

    def battery_callback(self, data):
        if (self.check_size('/sirio/energy/battery', len(data.data))):
            rospy.loginfo(f"Battery Status, SOC: {data.data[0]:.2f} %")
            rospy.loginfo(f"Battery Status, Temperature: {data.data[1]:.2f} °C")
            rospy.loginfo(f"Battery Status, Voltage: {data.data[2]:.2f} V")
            rospy.loginfo(f"Battery Status, Current: {data.data[3]:.2f} A")
            rospy.loginfo(f"Battery Status, Residual Capacity: {data.data[4]:.2f} mAh\n")

    def loadcell_1_callback(self,data):
        if (self.check_size('/sirio/systems/loadcell_1', len(data.data))):
            rospy.loginfo(f"Weight measured on loadcell 1 - sand sampling: {data.data[0]:.4f} g")
            rospy.loginfo(f"Raw mass data from loadcell 1: {data.data[1]:.4f}\n")   # 226
        
    def loadcell_2_callback(self,data):
        if (self.check_size('/sirio/systems/loadcell_2', len(data.data))):
            rospy.loginfo(f"Weight measured on loadcell 2 - rock sampling: {data.data[0]:.4f} g")
            rospy.loginfo(f"Raw mass data from loadcell 2: {data.data[1]:.4f}\n")   # 118
        
    def loadcell_3_callback(self,data):
        if (self.check_size('/sirio/systems/loadcell_3', len(data.data))):
            rospy.loginfo(f"Weight measured on loadcell 3 - drill sampling: {data.data[0]:.4f} g")
            rospy.loginfo(f"Raw mass data from loadcell 3: {data.data[1]:.4f}\n")

    def current_sensors_callback(self, data):

        if (self.check_size('/sirio/energy/current_sensors', len(data.data))):

            rospy.loginfo("")
            rospy.loginfo(f"Current Sensors Status, Converter,  sensor 1: {data.data[0]:.2f} A")
            rospy.loginfo(f"Current Sensors Status, Spare,      sensor 2: {data.data[1]:.2f} A")
            rospy.loginfo(f"Current Sensors Status, Traction,   sensor 3: {data.data[2]:.2f} A")
            rospy.loginfo(f"Current Sensors Status, Steer,      sensor 4: {data.data[3]:.2f} A\n")
            
            # # Update variables for offset computation
            # self.counter_sensing += 1
            # self.sensor_1 += data.data[0]
            # self.sensor_2 += data.data[1]
            # self.sensor_3 += data.data[2]
            # self.sensor_4 += data.data[3]
            
            # if (self.counter_sensing >= 20):
            #     self.counter_sensing = 0
            #     rospy.loginfo(f"Current sensor 1 offset: {self.sensor_1/20} A")
            #     rospy.loginfo(f"Current sensor 2 offset: {self.sensor_2/20} A")
            #     rospy.loginfo(f"Current sensor 3 offset: {self.sensor_3/20} A")
            #     rospy.loginfo(f"Current sensor 4 offset: {self.sensor_4/20} A\n")
            #     self.sensor_1 = 0
            #     self.sensor_2 = 0
            #     self.sensor_3 = 0
            #     self.sensor_4 = 0

    def run(self): 
        rate = rospy.Rate(10) # 10hz
    
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        monitor = RoverStatusMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
