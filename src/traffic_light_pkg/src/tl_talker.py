#!/usr/bin/env python3

#
#  *****************************************************************************************
#  * @file     tl_talker.py
#  * @author   RBRO/PJ-IU
#  * @version  V1.0
#  * @date     08-12-2020 drago
#  * @brief    This file contains the traffic light operation for the start, tl1,tl2 and tl3
#  *****************************************************************************************
#

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8
from itertools import cycle
from enum import IntEnum

class trafficlight(Node):
    # TrafficLightColor
    class Color(IntEnum):
        RED     = 0
        YELLOW  = 1
        GREEN   = 2

    def mirrorLight(self, number):
        if number == 0:
            return 2
        if number == 2:
            return 0
        return number

    
    def __init__(self):
        super().__init__('traffic_light_publisher_node')
        
        #Constants
        #Time for traffic light to change colors in seconds
        self.TL_interval = 0.5  # 500ms per step

        self.trafficlights = []
        #Create a new publisher, specify the topic name, type of message and queue size
        tlma = self.create_publisher(UInt8, '/automobile/trafficlight/master', 1)
        tlsl = self.create_publisher(UInt8, '/automobile/trafficlight/slave', 1)
        tlam = self.create_publisher(UInt8, '/automobile/trafficlight/antimaster', 1)
        tlst = self.create_publisher(UInt8, '/automobile/trafficlight/start', 1)
        
        self.trafficlights.append(tlma)
        self.trafficlights.append(tlsl)
        self.trafficlights.append(tlam)
        self.trafficlights.append(tlst)

        # Create timer with 10 Hz (0.1 seconds)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Traffic light cycle: RED -> YELLOW -> GREEN -> YELLOW -> RED
        self.pattern = [
            self.Color.RED,      # 0.0s - 0.5s
            self.Color.RED,      # 0.5s - 1.0s
            self.Color.RED,      # 1.0s - 1.5s
            self.Color.RED,      # 1.5s - 2.0s
            self.Color.RED,      # 2.0s - 2.5s
            self.Color.RED,      # 2.5s - 3.0s

            self.Color.YELLOW,   # 3.0s - 3.5s (transition to green)
            self.Color.YELLOW,   # 3.5s - 4.0s
            
            self.Color.GREEN,    # 4.0s - 4.5s
            self.Color.GREEN,    # 4.5s - 5.0s
            self.Color.GREEN,    # 5.0s - 5.5s
            self.Color.GREEN,    # 5.5s - 6.0s
            self.Color.GREEN,    # 6.0s - 6.5s
            self.Color.GREEN,    # 6.5s - 7.0s

            self.Color.YELLOW,   # 7.0s - 7.5s (transition to red)
            self.Color.YELLOW    # 7.5s - 8.0s (then back to RED)
        ]        
    
        # Cycles of patterns
        self.maincycle       = cycle(self.pattern)      
        self.last_time = self.get_clock().now().seconds_nanoseconds()[0] - self.TL_interval*2
        self.main_state = next(self.maincycle)

    #Function that publishes into the TL Topic the TL message (id and state)
    def sendState(self, id, state):
        Traffic_light = UInt8()
        Traffic_light.data = state

        self.trafficlights[id].publish(Traffic_light)

    def timer_callback(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_time > self.TL_interval:
            self.main_state = next(self.maincycle)
            self.last_time  = current_time

        # Send State for the two opposite traffic lights
        self.sendState(0, self.main_state)
        self.sendState(1, self.main_state)

        # Send State for traffic light with the opposite state of the previous ones
        self.sendState(2, self.mirrorLight(self.main_state))

        #Send State for the start semaphore
        self.sendState(3, self.mirrorLight(self.main_state))

def main(args=None):
    rclpy.init(args=args)
    nod = trafficlight()
    try:
        rclpy.spin(nod)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        nod.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
