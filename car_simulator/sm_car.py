from transitions import Machine
from car import Car
import pygame
import numpy as np
import math

class SM_Car(Car):
    # def __init__(self, car,states, transitions, initial):
    def __init__(self, car):

        self.machine = Machine(
            model=self,
            states=['idle','follow_path','brake'],
            transitions=[{'trigger': 'start', 'source': 'idle', 'dest': 'follow_path'},
                {'trigger': 'pause', 'source': 'follow_path', 'dest': 'brake'},
                {'trigger': 'abort', 'source': '*', 'dest': 'idle'},
                {'trigger': 'finish', 'source': 'follow_path', 'dest': 'idle'}],
            initial='idle'
        )
        self.car = Car(car[0],car[1],car[2])
        # self.state = 'idle'
        # self.transitions = transitions

        self.car_angle = car[2]  # Facing right initially
        self.speed = 0
        self.rotation_speed = 2
        self.steer_angle = 0
        self.brake_percent = 0
        
        #force
        self.force_motor = np.array([0.0,0.0,0.0])
        self.force_centri = np.array([0.0,0.0,0.0])
        self.force_drag = np.array([0.0,0.0,0.0])
        self.force_result = np.array([0.0,0.0,0.0])
        self.force_brake = 0
        self.force_brake_v = np.array([0.0,0.0,0.0])
        #acceleration
        self.acceleration = 0
        self.acceleration_m = np.array([0.0,0.0,0.0])
        self.acceleration_r = np.array([0.0,0.0,0.0])
        #velocity
        self.velocity = np.array([0.0,0.0,0.0])
        #position
        self.position = np.array([car[0],car[1],0])
        self.position_prev = np.array([0,0,0])
        #ICC @ Ackerman parameter
        self.radius_c = np.array([0.0,0.0,0.0])
        self.center_c = np.array([0.0,0.0,0.0])
        # limit
        self.max_speed = 2
        self.const_acceleration = 0.001
        self.max_steer = 50
        self.const_drag = 0.000001
        # DESIRED
        self.speed_desi = 0

        # size & mass
        self.mass = 100
        self.length = 50    #wheelbase
        self.width = 24

        #car image
        self.image = pygame.image.load('car_sedan_2.png') #original size 768x361

        self.init_set = True

        #state machine
        # self.stt_machine_ = Machine(model=self.car,states=states,transitions=transitions,initial=initial)
        # self.stt_machine_.on_enter_follow_path('on_enter_follow_path_')

        #waypoint
        self.waypoint = []
        self.waypoint_target = [0.0, 0.0, 0.0]
        self.waypoint_index = 0
        self.radius_wp = 10


    
    def set_waypoint(self,waypoint):
        self.waypoint = waypoint

    def get_waypoint(self, index):
        return self.waypoint[index]
    
    def get_waypoint_all(self):
        return self.waypoint

    def on_enter_follow_path(self):
        print("callback on_enter_follow_path")
        self.waypoint_target = self.waypoint[1]
        self.waypoint_index = 1
        # self.state = 'follow_path'

    def update(self):

        print("1. update")
        # Check state
        if (self.state == 'follow_path'):
            # reach position? calculating minimum tolerate distance to target
            a = self.waypoint_target[0]-self.position[0]    #x
            b = self.waypoint_target[1]-self.position[1]    #y
            radius = math.sqrt(math.pow(a,2)+math.pow(b,2))
            print(f"radius to wp: {radius:.2f}")
            if (radius < self.radius_wp):
                #change waypoint
                print(f"wp_index: {self.waypoint_index} length of waypoint: {len(self.waypoint)}")
                if(self.waypoint_index < len(self.waypoint)):  #still another waypoint to go
                    self.waypoint_index += 1
                    self.waypoint_target = self.waypoint[self.waypoint_index]
                    print("move to next waypoint")
                else:   #waypoints end
                    SM_Car.drive(0,0,0)
                    SM_Car.trigger('finish')
                    print("finish")
            else: #move
                #calculate turn
                # A = np.array([a,b])
                # B = np.array([math.cos(math.radians(self.car_angle)), math.sin(math.radians(self.car_angle))])

                # dot_product = np.dot(A, B)

                # # Calculate magnitudes (lengths of the vectors)
                # magnitude_A = np.linalg.norm(A)
                # magnitude_B = np.linalg.norm(B)

                # # Calculate angle in radians
                # angle_radians = np.arccos(dot_product / (magnitude_A * magnitude_B))

                # # Convert radians to degrees
                # angle_degrees = np.degrees(angle_radians)

                # print(f"vector angle: {angle_degrees:.2f} degrees, radian: {angle_radians}")
                # print(f"v car-->wp: {A}, v car_angle: {B}")

                # get angle of waypoint
                wp_angle = math.degrees(math.atan2(b,a))
                wp_angle %= 360                
                
                # print(f"distance to waypoint: {math.sqrt(math.pow(a,2)+math.pow(b,2)):.2f}")
                #print(f"waypoint target: {self.waypoint_target}")
                
                print(f"angle to waypoint: {wp_angle:.2f}")
                print(f"car_angle: {self.car_angle:.2f}")
                dif_angle = wp_angle - self.car_angle
                print(f"dif_angle: {dif_angle:.2f}")
                
                #check car angle
                if(self.car_angle >= 0 and self.car_angle < 90):    #Q1
                    if(wp_angle >= self.car_angle and wp_angle <= self.car_angle + 180):
                        self.turnRight(dif_angle)
                        print("Q1 right")
                    elif (wp_angle < self.car_angle or wp_angle > 180 + self.car_angle):
                        self.turnLeft(dif_angle)
                        print("Q1 left")
                    else:
                        print("unknown_1")
                elif(self.car_angle >= 90 and self.car_angle < 180):    #Q2
                    if(wp_angle >= self.car_angle and wp_angle <= self.car_angle + 180):
                        self.turnRight(dif_angle)
                        print("Q2 right")
                    elif (wp_angle < self.car_angle or wp_angle > 180 + self.car_angle):
                        self.turnLeft(dif_angle)
                        print("Q2 left")
                    else:
                        print("unknown_2")
                elif(self.car_angle >= 180 and self.car_angle < 270):    #Q3
                    if(wp_angle >= self.car_angle or wp_angle <= self.car_angle-180):
                        self.turnRight(dif_angle)
                        print("Q3 right")
                    elif (wp_angle < self.car_angle and wp_angle > self.car_angle-180):
                        self.turnLeft(dif_angle)
                        print("Q3 left")
                    else:
                        print("unknown_3")
                elif(self.car_angle >= 270 and self.car_angle < 360):    #Q4
                    if(wp_angle >= self.car_angle or wp_angle <= self.car_angle-180):
                        self.turnRight(dif_angle)
                        print("Q4 right")
                    elif (wp_angle < self.car_angle and wp_angle > self.car_angle-180):
                        self.turnLeft(dif_angle)
                        print("Q4 left")
                    else:
                        print("unknown_4")
                else:
                    print("unknown_5")

                # if(dif_angle <= 181 and dif_angle > 1):    #turn right
                   
                #     if(dif_angle > self.max_steer):
                #         self.steer_angle = self.max_steer
                #         print("right steer, positive diff, max")
                #     else:
                #         self.steer_angle = dif_angle
                #         print("right steer, positive diff")
                # elif((dif_angle > 181 and dif_angle < 360)):    #turn left
                    
                #     comp_angle = 360-dif_angle
                #     if(comp_angle > self.max_steer):
                #         self.steer_angle = -1*self.max_steer
                #         print("left steer, positive diff, max")
                #     else:
                #         self.steer_angle = -1*comp_angle
                #         print("left steer, positive diff")
                # elif(dif_angle < -1 and dif_angle >= -181):    #turn left
        
                #     if dif_angle < -1*self.max_steer:
                #         self.steer_angle = -1*self.max_steer
                #         print("left steer, negative diff, max")
                #     else:
                #         self.steer_angle = dif_angle
                #         print("left steer, negative diff")                        

                # elif(dif_angle < -181 and dif_angle > -360):    #turn right

                #     comp_angle = abs(360+dif_angle)
                #     if(comp_angle > self.max_steer):
                #         self.steer_angle = self.max_steer
                #         print("right steer, negative diff, max")
                #     else:
                #         self.steer_angle = comp_angle
                #         print("right steer, negative diff")                      

                SM_Car.drive(self,2,self.steer_angle,0)

                # print(f"steer_angle: {self.steer_angle:.2f}")

            print(f"waypoint index: {self.waypoint_index}")
            

        # Update position based on speed and car angle
        # self.position_prev = self.position
        # print("2. update")


        #acceleration
        Car.calcAccelerationMag(self)
        Car.calcCarAcceleration(self)
        Car.calcForce(self)
        Car.calcCentripetal(self)
        Car.calcForceDrag(self)
        Car.calcForceResultant(self)
        Car.calcAccelerationResult(self)

        # forward straight motion
        if (abs(self.steer_angle) < 0.1): #forward straight motion
            self.velocity = np.add(self.velocity,self.acceleration_r)
            self.position = np.add(self.position,self.velocity)
            self.radius_c.fill(0)
            self.center_c.fill(0)
            print("straight motion")

        # TURN
        else:
            print("turn motion")
            vpa = np.add(self.velocity, self.acceleration_r)
            # print(f"vpa: {vpa}")
            speed = np.linalg.norm(vpa)
            # print(f"speed: {speed}")

            # print(f"cos({math.radians(self.car_angle):.2f}) = {math.cos(math.radians(self.car_angle))}")
            self.velocity[0] = speed*math.cos(math.radians(self.car_angle))
            # print(f"velocity[0]: {self.velocity[0]}")
            self.velocity[1] = speed*math.sin(math.radians(self.car_angle))

            # self.velocity = np.add(self.velocity, self.acceleration_r)

            self.position = np.add(self.position, self.velocity)

            #heading change
            self.car_angle += math.degrees((speed/self.length)*math.tan(math.radians(self.steer_angle)))
            self.car_angle %= 360

        print(f"position: {self.position}")
        print(f"velocity: {self.velocity}")
        print(f"acceleration: {self.acceleration_r}")
        # print(f"car_angle: {self.car_angle:.2f}")
    
    def turnRight(self,dif_a):
        if(abs(dif_a) < self.max_steer):
            self.steer_angle = abs(dif_a)
        else:
            self.steer_angle = self.max_steer
    
    def turnLeft(self,dif_a):
        if(abs(dif_a) < self.max_steer):
            self.steer_angle = -1*abs(dif_a)
        else:
            self.steer_angle = -1*self.max_steer
    
