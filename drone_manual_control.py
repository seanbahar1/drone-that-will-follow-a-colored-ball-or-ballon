import dronekit
from time import sleep
import keyboard 
from pymavlink import mavutil

#-- this class is a class that contains all the functions of the drone controler
class droneCommands:
    def __init__(self, COM_PORT_IP):
        # Connect to UDP endpoint.
        self.gnd_speed = 5 #[m/s]
        #-- if we using ip : '127.0.0.1:14550' #-- if using COM :: manual find : 'COMx' usualy COM3
        self.vehicle = dronekit.connect(COM_PORT_IP, wait_ready=False, baud=57600 ) 
        self.drone_running = True
        print("Mode: %s" % self.vehicle.mode.name)
        self.flight_mode_with_or_without_reset = False
        self.sleep_before_reset = 0.7
        self.requestAmount = 4
        self.requestDelay = 0.1
    #-- this function is the take off function
    def Fmode(self):
        return self.vehicle.mode
    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        print ("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print (" Waiting for vehicle to initialise...")
            sleep(1)

        print ("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode    = dronekit.VehicleMode("GUIDED")
        self.vehicle.armed   = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print (" Waiting for arming...")
            sleep(1)

        print ("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print (" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            #Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                
                print ("Reached target altitude")
                break
        sleep(1)

    #-- this function sends mavlink message
    def set_velocity_body(self,vx,vy,vz):
        message = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities -> 1 = ignore velocity bit
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
        self.vehicle.send_mavlink(message)
        self.vehicle.flush()
       

        #--- all the functions below will send a message to the drone and command will run untill the command to reset velocity is sent ---#

    #-- this function moved the drone forward
    def up(self):
        self.set_velocity_body(self.gnd_speed,0,0)

    #-- this function moved the drone backwards
    def down(self):
        self.set_velocity_body(-self.gnd_speed,0,0)

    #-- this function moved the drone left
    def left(self):
        self.set_velocity_body(0,-self.gnd_speed,0)

    #-- this function moved the drone right
    def right(self):
        self.set_velocity_body(0,self.gnd_speed,0)

    #-- this function moves the drone forward + right
    def up_right(self):
        self.set_velocity_body(self.gnd_speed,self.gnd_speed,0)

    #-- this function moves the drone forwards + left
    def up_left(self):
        self.set_velocity_body(self.gnd_speed,-self.gnd_speed,0)
    
    #-- this function moves the drone backwards + right
    def down_right(self):
        self.set_velocity_body(-self.gnd_speed,self.gnd_speed,0)
    
    #-- this function moves the drone backwards + left
    def down_left(self):
        self.set_velocity_body(-self.gnd_speed,-self.gnd_speed,0)
    
    #-- this function moved the drone forward and resets velocity 
    def up_r(self):
        self.set_velocity_body(self.gnd_speed,0,0)
        sleep(self.sleep_before_reset)
        self.reset_velocity()

    #-- this function moved the drone backwards and resets velocity
    def down_r(self):
        self.set_velocity_body(-self.gnd_speed,0,0)
        sleep(self.sleep_before_reset)
        self.reset_velocity()

    #-- this function moved the drone left and resets velocity
    def left_r(self):
        self.set_velocity_body(0,-self.gnd_speed,0)
        sleep(self.sleep_before_reset)
        self.reset_velocity()

    #-- this function moved the drone right and resets velocity
    def right_r(self):
        self.set_velocity_body(0,self.gnd_speed,0)
        sleep(self.sleep_before_reset)
        self.reset_velocity()

    #-- this function changes the drone mode to rtl
    def mode_RTL(self):
        print('r was pressed -> setting vehicle to RTL MODE')
        self.vehicle.mode = dronekit.VehicleMode("RTL")
        print('RTL ON -> returning to the start point \n', '[system]: ending the program')
        self.drone_running = False

    #-- test if loiter is possible   
    #-- this function changes the drone mode to loiter
    def mode_LOITER(self):
        print('L was pressed -> setting vehicle to LOITER MODE')
        self.vehicle.mode = dronekit.VehicleMode("LOITER")

    #-- this function resets the velocity of the drone body. stops the drone in place i believe
    def reset_velocity(self):
        self.set_velocity_body(0,0,0)

    def printTest(self):
        print(self.vehicle.commands)

""" 
the code below is only for the testing part::: not main module
"""
if __name__ == "__main__":
    com = "COM4"
    control = True
    if not control:
        drone = droneCommands(com) #-> this will be changed manually at the testing spot
    #---- MAIN FUNCTION    
    #---- Only Main module 
    while(not control):
        print(drone.Fmode())
    #-- creating an instance of the class for the drone control and commands                      
    drone = droneCommands(com) #-> this will be changed manually at the testing spot
    print('this is the main module page which means its the manual control program\n')
    while drone.drone_running == True:
        #-- manual control test
        try:
            #-- RTL
            if keyboard.is_pressed('r'):
                drone.mode_RTL()
            
            #-- Dangerous: -> changes to LOITER
            #-- only used when the drone is safe or near a safe place to crash. P.S: idk what it will do to the drone so yeah...
            elif keyboard.is_pressed('l'):
                drone.mode_LOITER()

            #this will change the controller mode: example: reset -> not reset
            elif keyboard.is_pressed('m'):
                if(drone.flight_mode_with_or_without_reset):
                    drone.flight_mode_with_or_without_reset = False
                else:
                    drone.flight_mode_with_or_without_reset = True

            elif keyboard.is_pressed('b'):
                drone.arm_and_takeoff(2)
            elif keyboard.is_pressed('k'):
                drone.vehicle.mode    = dronekit.VehicleMode("GUIDED")

            #-- FORWARD
            elif keyboard.is_pressed('w'):
                if not drone.flight_mode_with_or_without_reset:
                    drone.up()
                else:
                    drone.up_r()

            #-- BACKWARD
            elif keyboard.is_pressed('s'):
                if not drone.flight_mode_with_or_without_reset:
                    drone.down()
                else:
                    drone.down_r()

            #-- LEFT
            elif keyboard.is_pressed('a'):
                if not drone.flight_mode_with_or_without_reset:
                    drone.left()
                else:
                    drone.left_r()

            #-- RIGHT
            elif keyboard.is_pressed('d'):
                if not drone.flight_mode_with_or_without_reset:
                    drone.right()
                else:
                    drone.right_r()
        except:
            print("useless key was pressed :)")  # if user pressed a key other than the given key the loop will break
        sleep(1)
