# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""
import logging
import time
from threading import Timer
import datetime as dt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

import math
import numpy as np
import os
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


# Class that implements our own Position Commander.
# It inherits from the High Level Position Commander.
class PCown(PositionHlCommander):

    # Method that resets the kalman filter
    def reset_est(self):
        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
    
    # Method for flying to a point, while changing the yaw
    def goto_with_yaw(self, x, y, z, yaw, velocity=None):
        z = self._height(z)

        dx = x - self._x
        dy = y - self._y
        dz = z - self._z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        # TODO: adjust the duration depending on the distance to the goal
        # or the velocity we want to fly at.
        # if distance > 0.0:
        duration_s = 1 # distance / self._velocity(velocity)
        print(f"x: {x}, y: {y}, z: {z}, yaw: {yaw}, duration: {duration_s}")
        self._hl_commander.go_to(x, y, z, yaw, duration_s)
        time.sleep(duration_s)

        self._x = x
        self._y = y
        self._z = z
    
    # Overwrite the enter function to avoid taking off
    def __enter__(self):
        # self.take_off()
        return self



class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link id and disconnects after 5s.
    """
    START = 0
    SMALL_STEP = 0.01
    SAFETY_DISTANCE = 0.05
    FORWARD_DISTANCE = 2.75
    GETBACK_DISTANCE = 0.6
    LEFT_END = 1.3
    GETBACK_LEFT_END = 0.50
    RIGHT_END = -1.3
    GETBACK_RIGHT_END = -0.50
    FRONT_SWEEP_DIST = 0.3
    PAD_MIDDLE = 0.20
    THRESH = 0.02
    SWEEPING_SPEED = 0.6

    def __init__(self, link_id):
        """ Initialize and run the example with the specified link_id """

        self.count = 0

        # Initialize cf object
        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # Initialize log variable
        self.logs = np.zeros([100000,3])

        # Fly a square
        self.fly_trajectory(link_id)

    def _connected(self, link_id):

        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        self._lg_stab.add_variable('stateEstimate.x', 'float')  # estimated X coordinate
        self._lg_stab.add_variable('stateEstimate.y', 'float')  # estimated Y coordinate
        self._lg_stab.add_variable('stateEstimate.z', 'float')  # estimated Z coordinate

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        # try:
        self._cf.log.add_config(self._lg_stab)
        # This callback will receive the data
        self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
        # This callback will be called on errors
        self._lg_stab.error_cb.add_callback(self._stab_log_error)
        # Start the logging
        self._lg_stab.start()
        # except KeyError as e:
        #     print('Could not start log configuration,'
        #           '{} not found in TOC'.format(str(e)))
        # except AttributeError:
        #     print('Could not add Stabilizer log config, bad configuration.')
   
    def is_close(self, up_distance):
        MIN_DISTANCE = 0.5
        if up_distance is None:
            return False
        else:
            return up_distance < MIN_DISTANCE

    def pad_detection(self, down_distance, current_state, general_state):
        pad_detection_threshold = 0.26
        if down_distance < pad_detection_threshold:
            if general_state == 'landing_region':
                return 'landing'
            elif general_state == 'final_landing_region':
                return 'final_landing'
        else:
            return current_state

    def edge_detection(self, zvalue):

        edge_threshold = 0.275
        if zvalue < edge_threshold:
            return 1
        else:
            return 0
        
    def go_straight(self, pc, multiranger, state):

        desired_x = 0.0
        # Obstacle detected in front
        if self.is_close(multiranger.front):
            # Get y position when obstacle is seen
            obstacle_y = self.get_state_estimate()[1]
            while self.is_close(multiranger.front):

                #If obstacle is on right side of map, avoid from left side
                if  obstacle_y <= 0:
                    #If there is no obstacle on the left
                    if not self.is_close(multiranger.left):
                        pc.left(self.SMALL_STEP)
                    else: obstacle_y = 1.0

                #Otherwise avoid from right side
                else:
                    #If there is no obstacle on the right 
                    if not self.is_close(multiranger.right):
                        pc.right(self.SMALL_STEP)
                    else: obstacle_y = -1.0

            #Safety distance to avoid obstacle
            if obstacle_y >= 0:
                pc.left(self.SAFETY_DISTANCE)

            else:
                pc.right(self.SAFETY_DISTANCE)

        # Small forward increments
        pc.forward(self.SMALL_STEP, velocity = self.SWEEPING_SPEED)

        #When drone has gone forward for a long enough distance
        if self.get_state_estimate()[0] >= self.FORWARD_DISTANCE:

            # Enter sweeping mode and store x position of drone
            state = 'sweeping_left'
            x_position = self.get_state_estimate()[0]
            desired_x = self.FORWARD_DISTANCE

        return state, desired_x

    def sweeping_left(self, pc, multiranger, state, desired_x, left, x_position):

        # If obstacle is seen on the left
        if self.is_close(multiranger.left):
            x_before_avoide = self.get_state_estimate()[0]

            # Avoid until not seen + safety distance
            while self.is_close(multiranger.left):
                pc.forward(self.SMALL_STEP)
            pc.forward(self.SAFETY_DISTANCE)
            while not self.is_close(multiranger.back):
                pc.left(self.SMALL_STEP)
            while self.is_close(multiranger.back):
                pc.left(self.SMALL_STEP)
            pc.left(self.SAFETY_DISTANCE)
            
            #Store new x position
            y_after_avoide = self.get_state_estimate()[1]
            pc.go_to(x_before_avoide, y_after_avoide + self.SAFETY_DISTANCE, velocity=0.1)

        # Small distance increments
        pc.left(self.SMALL_STEP, velocity = self.SWEEPING_SPEED)

        # When reached left end of map
        if self.get_state_estimate()[1] >= self.LEFT_END:

            # Go into front sweep mode
            state = 'sweeping_front'

            # Readjust position
            pc.go_to(desired_x, self.LEFT_END, velocity=0.1)
            x_position = self.get_state_estimate()[0]
            left = False
            # Update desired x position
            desired_x = desired_x + self.FRONT_SWEEP_DIST
        
        return state, desired_x, left, x_position

    def sweeping_right(self, pc, multiranger, state, desired_x, left, x_position):

        # If obstacle is seen on the right
        if self.is_close(multiranger.right):
            x_before_avoide = self.get_state_estimate()[0]

            # Avoid until not seen + safety distance
            while self.is_close(multiranger.right):
                pc.forward(self.SMALL_STEP)
            pc.forward(self.SAFETY_DISTANCE)
            while not self.is_close(multiranger.back):
                pc.right(self.SMALL_STEP)
            while self.is_close(multiranger.back):
                pc.right(self.SMALL_STEP)
            pc.right(self.SAFETY_DISTANCE)
            
            #Store new x position
            y_after_avoide = self.get_state_estimate()[1]
            pc.go_to(x_before_avoide, y_after_avoide - self.SAFETY_DISTANCE, velocity=0.1)

        # Small increment to the right
        pc.right(self.SMALL_STEP, velocity=self.SWEEPING_SPEED)

        # When reaching right end of map
        if self.get_state_estimate()[1] <= self.RIGHT_END:

            # Enter sweeping front mode
            state = 'sweeping_front'

            # Readjust position
            pc.go_to(desired_x, self.RIGHT_END, velocity=0.1)
            x_position = self.get_state_estimate()[0]
            left = True
            # Update desired x position
            desired_x = desired_x + self.FRONT_SWEEP_DIST

        return state, desired_x, left, x_position

    def sweeping_front(self, pc, multiranger, state, desired_x, left, x_position):
        # If obstacle is detected in front
        if self.is_close(multiranger.front):
            # Avoid until not seen anymore
            while self.is_close(multiranger.front):
                # By the correct side + safety distance
                if left:
                    pc.left(self.SMALL_STEP)
                else:
                    pc.right(self.SMALL_STEP)
            if left:
                pc.left(self.SAFETY_DISTANCE)
            else:
                pc.right(self.SAFETY_DISTANCE)

        # Small forward increment
        pc.forward(self.SMALL_STEP)

        # When advanced from chosen distance, either on left or right end of map
        if left:
            if self.get_state_estimate()[0]-x_position >= self.FRONT_SWEEP_DIST:

                # Enter sweeping left motion
                state = 'sweeping_left'
                x_position = self.get_state_estimate()[0]
                # Readjust position
                pc.go_to(desired_x, self.RIGHT_END, velocity=0.1)

        if not left:
            if self.get_state_estimate()[0]-x_position >= self.FRONT_SWEEP_DIST:

                # Enter sweeping right mode
                state = 'sweeping_right'
                x_position = self.get_state_estimate()[0]
                # Readjust position
                pc.go_to(desired_x, self.LEFT_END, velocity=0.1)
            
        return state, desired_x, left, x_position
        
    def landing(self, pc, multiranger, state, general_state, move):

        # Moving front little by little until end of pad is found with edge detection
        if move == 'front':
            pc.forward(self.SMALL_STEP, velocity=0.1)
            pad_end = self.get_state_estimate()
            get_back_y_pos = 0.0

            if self.edge_detection(multiranger.down):
                # Go back to reach x middle of pad
                while state == 'landing':
                    pc.back(self.SMALL_STEP, velocity=0.1)
                    if self.edge_detection(multiranger.down):
                        pc.go_to(pad_end[0]-self.PAD_MIDDLE, pad_end[1], velocity=0.05)
                        # Finally land
                        pc.land(0.15)
                        time.sleep(2.0)
                        get_back_y_pos = self.get_state_estimate()
                        if general_state == 'final_landing_region':
                            state = 'done'
                            keep_flying = False
                            break
                        pc.take_off(0.3, 0.3)
                        general_state = 'getback'
                        time.sleep(1.0)
                        state = 'get_back_y'
                
        return state, general_state, get_back_y_pos

    def get_back_y(self, pc, multiranger, state, desired_x, get_back_y_pos):

        if get_back_y_pos[1]>=0:

            # If obstacle is seen on the right
            if self.is_close(multiranger.right):
                # Avoid until not seen anymore + safety distance
                while self.is_close(multiranger.right):
                    pc.back(self.SMALL_STEP)
                pc.back(self.SAFETY_DISTANCE)
                desired_x = self.get_state_estimate()[0]

            # Small increment to the right
            pc.right(self.SMALL_STEP, velocity=self.SWEEPING_SPEED)

            # When reaching y middle of map
            if self.get_state_estimate()[1] <= self.START:
                # Enter getting back in x mode
                state = 'get_back_x'

        if get_back_y_pos[1] < 0:

            # If obstacle is seen on the left
            if self.is_close(multiranger.left):

                # Avoid until not seen anymore + safety distance
                while self.is_close(multiranger.left):
                    pc.back(self.SMALL_STEP)
                pc.back(self.SAFETY_DISTANCE)
                desired_x = self.get_state_estimate()[0]

            # Small increment to the left
            pc.left(self.SMALL_STEP, velocity=self.SWEEPING_SPEED)

            # When reaching y center of map
            if self.get_state_estimate()[1] > self.START:
                # Enter getting back in x mode
                state = 'get_back_x'
            
        return state, desired_x

    def get_back_x(self, pc, multiranger, state, left, x_position, desired_x):

        if self.is_close(multiranger.back):
            # Get y position when obstacle is seen
            obstacle_y = self.get_state_estimate()[1]
            while self.is_close(multiranger.back):

                #If obstacle is on right side of map, avoid from left side
                if  obstacle_y <= 0:
                    #If there is no obstacle on the left 
                    if not self.is_close(multiranger.left):
                        pc.left(self.SMALL_STEP)
                    else: obstacle_y = 1.0

                #Otherwise avoid from right side
                else:
                    # If there is no obstacle on the right 
                    if not self.is_close(multiranger.right):
                        pc.right(self.SMALL_STEP)
                    else: obstacle_y = -1.0

            #Safety distance to avoid obstacle
            if obstacle_y >= 0:
                pc.left(self.SAFETY_DISTANCE)


            else:
                pc.right(self.SAFETY_DISTANCE)

        pc.back(self.SMALL_STEP, velocity = self.SWEEPING_SPEED)

        if self.get_state_estimate()[0] <= self.GETBACK_DISTANCE:
            # Enter sweeping mode and store x position of drone
            state = 'getback_sweeping_left'
            left = True
            x_position = self.get_state_estimate()[0]
            desired_x = self.get_state_estimate()[0]

        return state, left, x_position, desired_x

    def getback_sweeping_left(self, pc, multiranger, state, desired_x, left, x_position):
        # If obstacle is seen on the left
        if self.is_close(multiranger.left):
            x_before_avoide = self.get_state_estimate()[0]

            # Avoid until not seen + safety distance
            while self.is_close(multiranger.left):
                pc.back(self.SMALL_STEP)
            pc.back(self.SAFETY_DISTANCE)
            while not self.is_close(multiranger.front):
                pc.left(self.SMALL_STEP)
            while self.is_close(multiranger.front):
                pc.left(self.SMALL_STEP)
            pc.left(self.SAFETY_DISTANCE)
            
            #Store new x position
            y_after_avoide = self.get_state_estimate()[1]
            pc.go_to(x_before_avoide, y_after_avoide + self.SAFETY_DISTANCE, velocity=0.1)

        # Small distance increments
        pc.left(self.SMALL_STEP, velocity = self.SWEEPING_SPEED)

        # When reached left end of map
        if self.get_state_estimate()[1] >= self.GETBACK_LEFT_END:

            # Go into back sweep mode
            state = 'sweeping_back'
            # Readjust position
            x_position = self.get_state_estimate()[0]
            left = False
            # Update desired x position
            desired_x = desired_x - self.FRONT_SWEEP_DIST
        
        return state, desired_x, left, x_position

    def getback_sweeping_right(self, pc, multiranger, state, desired_x, left, x_position):
        # If obstacle is seen on the right
        if self.is_close(multiranger.right):
            x_before_avoide = self.get_state_estimate()[0]

            # Avoid until not seen + safety distance
            while self.is_close(multiranger.right):
                pc.back(self.SMALL_STEP)
            pc.back(self.SAFETY_DISTANCE)
            while not self.is_close(multiranger.front):
                pc.right(self.SMALL_STEP)
            while self.is_close(multiranger.front):
                pc.right(self.SMALL_STEP)
            pc.right(self.SAFETY_DISTANCE)
            
            #Store new x position
            y_after_avoide = self.get_state_estimate()[1]
            pc.go_to(x_before_avoide, y_after_avoide - self.SAFETY_DISTANCE, velocity=0.1)

        # Small increment to the right
        pc.right(self.SMALL_STEP, velocity=self.SWEEPING_SPEED)

        # When reaching right end of map
        if self.get_state_estimate()[1] <= self.GETBACK_RIGHT_END:

            # Enter sweeping front mode
            state = 'sweeping_back'

            # Readjust position
            x_position = self.get_state_estimate()[0]
            left = True
            # Update desired x position
            desired_x = desired_x - self.FRONT_SWEEP_DIST
        
        return state, desired_x, left, x_position

    def sweeping_back(self, pc, multiranger, state, desired_x, left, x_position):
        # If obstacle is detected in front
        if self.is_close(multiranger.back):

            # Avoid until not seen anymore
            while self.is_close(multiranger.back):
                # By the correct side + safety distance
                if left:
                    pc.left(self.SMALL_STEP)
                else:
                    pc.right(self.SMALL_STEP)
            if left:
                pc.left(self.SAFETY_DISTANCE)
            else:
                pc.right(self.SAFETY_DISTANCE)

        # Small backward increment
        pc.back(self.SMALL_STEP)

        # When advanced from chosen distance, either on left or right end of map
        if left:
            if x_position - self.get_state_estimate()[0] >= self.FRONT_SWEEP_DIST/2:

                # Enter sweeping left motion
                state = 'getback_sweeping_left'
                x_position = self.get_state_estimate()[0]

        if not left:
            if x_position - self.get_state_estimate()[0] >= self.FRONT_SWEEP_DIST/2:

                # Enter sweeping right mode
                state = 'getback_sweeping_right'
                x_position = self.get_state_estimate()[0]
            
        return state, desired_x, left, x_position

    def final_landing(self, pc, multiranger, move, state, keep_flying):
        # Moving back little by little until end of pad is found with edge detection
        if move == 'back':
            pc.back(self.SMALL_STEP, velocity=0.1)
            pad_end = self.get_state_estimate()

            if self.edge_detection(multiranger.down):
                # Go back to reach x middle of pad
                while state == 'final_landing':
                    pc.forward(self.SMALL_STEP, velocity=0.1)
                    if self.edge_detection(multiranger.down):
                        pc.go_to(pad_end[0] + self.PAD_MIDDLE, pad_end[1], velocity=0.05)
                        # Finally land
                        pc.land(0.15)
                        time.sleep(2.0)
                        get_back_y_pos = self.get_state_estimate()
                        keep_flying = False
                        state = 'done'
                
        return keep_flying, state
        
    def get_state_estimate(self):
        state_estimate = self.logs[self.logs!=0]
        return state_estimate[-3:]
        #state_estimate = state_estimate.reshape(len(state_estimate) // 3, -1)[-1]
        #return state_estimate
            

    def fly_trajectory(self, id):
        """ Example of simple logico to make the drone fly in a 
        trajectory at fixed speed"""
        state = 'go_straight'
        general_state = 'takeoff_region'
        x_position = 0
        desired_x = 0

        FORWARD_DISTANCE = 2.75
        GETBACK_DISTANCE = 0.6
        PAD_MIDDLE = 0.18
        THRESH = 0.02
        THRESH_LANDING = 0.005

        keep_flying = True
        z_values = []

        # Sync with drone
        with SyncCrazyflie(id, cf=self._cf) as scf:
            # Send position commands
            time.sleep(1.0)
            with Multiranger(scf) as multiranger:
                with PCown(scf, default_velocity=0.3, default_height=0.3) as pc:
                    time.sleep(2)
                    pc.reset_est()
                    time.sleep(1)
                    pc.take_off(0.3, 0.25)
                    time.sleep(1.0)
                    left = True

                    while keep_flying:

                        z_values.append(multiranger.down)

                        # Force stop flying with hand
                        if self.is_close(multiranger.up):
                                keep_flying = False
                                break

                        # Go forward to get to sweeping zone
                        if state == 'go_straight':
                            state, desired_x = self.go_straight(pc, multiranger, state)

                        # Sweeping map from the left
                        if state == 'sweeping_left':
                            state, desired_x, left, x_position = self.sweeping_left(pc, multiranger, state, desired_x, left, x_position)
                          
                        # Sweeping map from the right
                        if state == 'sweeping_right':
                            state, desired_x, left, x_position = self.sweeping_right(pc, multiranger, state, desired_x, left, x_position)
                            
                        # Sweeping front mode
                        if state == 'sweeping_front':
                            state, desired_x, left, x_position = self.sweeping_front(pc, multiranger, state, desired_x, left, x_position)

                        # If we are in the region where landing pads can be found
                        if general_state == 'landing_region' or general_state == 'final_landing_region':
                            # Start the pad detection mode
                            state = self.pad_detection(multiranger.down, state, general_state)

                        # Start being in landing region only when we went forward for long enough
                        elif general_state == 'takeoff_region' and self.get_state_estimate()[0] >= FORWARD_DISTANCE:
                            general_state = 'landing_region'
                        
                        # Start being in landing region when going back to the original take off pad 
                        elif general_state == 'getback' and self.get_state_estimate()[0] <= GETBACK_DISTANCE:
                            general_state = 'final_landing_region'

                        # When we are during landing motion
                        if state == 'landing':
                            
                            move = 'front'
                            # Store pad edge position
                            pad_end = self.get_state_estimate()
                            # Go to middle with respect to where it was coming from
                            if left:
                                pc.go_to(pad_end[0] + THRESH_LANDING, pad_end[1] + PAD_MIDDLE, velocity=0.05)
                            else:
                                pc.go_to(pad_end[0] + THRESH_LANDING, pad_end[1] - PAD_MIDDLE, velocity=0.05)
                            time.sleep(0.5)

                            while state == 'landing':
                                state, general_state, get_back_y_pos = self.landing(pc, multiranger, state, general_state, move)

                        # Going back on the y axis to face the first takeoff pad
                        if state == 'get_back_y':
                            state, desired_x = self.get_back_y(pc, multiranger, state, desired_x, get_back_y_pos)
                            
                        # Going forward toward the final landing pad
                        if state == 'get_back_x':
                            state, left, x_position, desired_x = self.get_back_x(pc, multiranger, state, left, x_position, desired_x)
                            
                        # Sweeping map from the left
                        if state == 'getback_sweeping_left':
                            state, desired_x, left, x_position = self.getback_sweeping_left(pc, multiranger, state, desired_x, left, x_position)                            

                        # Sweeping map from the right
                        if state == 'getback_sweeping_right':
                            state, desired_x, left, x_position = self.getback_sweeping_right(pc, multiranger, state, desired_x, left, x_position)
 
                        # Sweeping front mode
                        if state == 'sweeping_back':
                            state, desired_x, left, x_position = self.sweeping_back(pc, multiranger, state, desired_x, left, x_position)

                        # Final landing
                        if state == 'final_landing':

                            move = 'back'
                            # Store pad edge position
                            pad_end = self.get_state_estimate()
                            # Go to middle with respect to where it was coming from
                            if left:
                                pc.go_to(pad_end[0] - THRESH_LANDING, pad_end[1] + PAD_MIDDLE, velocity=0.05)
                            else:
                                pc.go_to(pad_end[0] - THRESH_LANDING, pad_end[1] - PAD_MIDDLE, velocity=0.05)
                            time.sleep(0.5)

                            while state == 'final_landing':
                                keep_flying, state = self.final_landing(pc, multiranger, move, state, keep_flying)
                                                


        self._disconnected

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        
        # Save info into log variable
        for idx, i in enumerate(list(data)):
            self.logs[self.count][idx] = data[i]

        self.count += 1

    def _connection_failed(self, link_id, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_id, msg))
        self.is_connected = False

    def _connection_lost(self, link_id, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_id, msg))

    def _disconnected(self, link_id):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_id)
        self.is_connected = False
        
        # Get timestamp
        filename = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv")
        print('file saved to: '+filename)
        # Save log to file
        if not os.path.exists('logs'):
            os.makedirs('logs')
        filepath = os.path.join(os.getcwd(),'..','logs',filename)
        #print(self.logs)
        #np.savetxt(filepath, self.logs, delimiter=',')


def connect_to_first_found():
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        return
    else:
        print('No Crazyflies found, cannot run example')


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # uncomment to connect to the first available drone
    # id = connect_to_first_found()
    # uncomment to connect to the selected drone
    id = 'radio://0/88/2M/E7E7E7E7F0'

    # Run example class
    if id is not None:
        le = LoggingExample(id)
        # The Crazyflie lib doesn't contain anything to keep the application alive,
        # so this is where your application should do something. In our case we
        # are just waiting until we are disconnected.
        while le.is_connected:
            time.sleep(1)

    else: 
        print('Please provide a valid ID')