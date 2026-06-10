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
            states=['idle','follow_path','brake','move','1_onBoomGate',
                    '3_obstruction','4_onRoundabout','5_inTunnel','6_onHill'
                    '2_onBumper','7_onTrafficLight','8_parallelParking',
                    '9_perpendicularParking'],
            transitions=[{'trigger': 'start', 'source': 'idle', 'dest': 'follow_path'},
                {'trigger': 'pause', 'source': 'follow_path', 'dest': 'brake'},
                {'trigger': 'abort', 'source': '*', 'dest': 'idle'},
                {'trigger': 'finish', 'source': 'follow_path', 'dest': 'idle'},
                
                {'trigger': 'begin', 'source': 'idle', 'dest': 'move'},
                {'trigger': 'move1', 'source': 'move', 'dest': '1_onBoomGate'},
                {'trigger': 'chkpt1', 'source': '1_onBoomGate', 'dest': 'move'},
                {'trigger': 'move2', 'source': 'move', 'dest': '3_obstruction'},
                {'trigger': 'chkpt2', 'source': '3_obstruction', 'dest': 'move'},
                {'trigger': 'move3', 'source': 'move', 'dest': '4_onRoundabout'},
                {'trigger': 'chkpt3', 'source': '4_onRoundabout', 'dest': 'move'},
                {'trigger': 'move4', 'source': 'move', 'dest': '5_inTunnel'},
                {'trigger': 'chkpt4', 'source': '5_inTunnel', 'dest': 'move'},
                {'trigger': 'move4', 'source': 'move', 'dest': '6_onHill'},
                {'trigger': 'chkpt5', 'source': '6_onHill', 'dest': 'move'},
                {'trigger': 'move5', 'source': 'move', 'dest': '2_onBumper'},
                {'trigger': 'chkpt6', 'source': '2_onBumper', 'dest': 'move'},
                {'trigger': 'move6', 'source': 'move', 'dest': '7_onTrafficLight'},
                {'trigger': 'chkpt7', 'source': '7_onTrafficLight', 'dest': 'move'},
                {'trigger': 'move7', 'source': 'move', 'dest': '1_onBoomGate'},
                {'trigger': 'chkpt8', 'source': '1_onBoomGate', 'dest': 'move'},
                {'trigger': 'move8', 'source': 'move', 'dest': '3_obstruction'},
                {'trigger': 'chkpt9', 'source': '3_obstruction', 'dest': 'move'},
                {'trigger': 'move9', 'source': 'move', 'dest': '4_onRoundabout'},
                {'trigger': 'chkpt10', 'source': '4_onRoundabout', 'dest': 'move'},
                {'trigger': 'move10', 'source': 'move', 'dest': '8_parallelParking'},
                {'trigger': 'chkpt11', 'source': '8_parallelParking', 'dest': 'move'},
                {'trigger': 'move11', 'source': 'move', 'dest': '9_perpendicularParking'},
                {'trigger': 'chkpt12', 'source': '9_perpendicularParking', 'dest': 'idle'},
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
        self.brake_input = 0 #in percentage
        self.force_brake_velocity_control = 0

        #force (internal)
        self.force_motor_v = np.array([0.0,0.0,0.0])
        self.force_brake = 0
        self.force_brake_v = np.array([0.0,0.0,0.0])
        self.force_centri = np.array([0.0,0.0,0.0])
        self.force_car_v = np.array([0.0,0.0,0.0])
        #force (external)
        self.force_drag = np.array([0.0,0.0,0.0])
        self.force_result_v = np.array([0.0,0.0,0.0])

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
        self.max_speed = 5 
        self.const_acceleration = 0.05
        self.max_steer = 45
        self.brake_coef = 7
        self.const_drag = 0.0001
        # DESIRED
        self.speed_desi = 0

        # size & mass
        self.mass = 100
        self.length = 50    #wheelbase
        self.width = 24

        #car image
        self.image = pygame.image.load('car_sedan_2.png') #original size 768x361
        self.init_set = True
        self.show_state = True

        #waypoint
        self.waypoint = []
        self.waypoint_target = [0.0, 0.0, 0.0]
        self.waypoint_index = 0
        self.radius_wp = 10

        #carbot
        self.checkpoint = 0
        self.ver_waypoint2 = True
        self.waypoint2 = {}



    
    def set_waypoint(self,waypoint):
        if self.ver_waypoint2:
            self.waypoint2 = waypoint
        else:
            self.waypoint = waypoint

    def set_waypoint2(self,waypoint):
        self.waypoint2 = waypoint

    def get_waypoint(self, index):
        if self.ver_waypoint2:
            return [self.waypoint2[index]["x"],self.waypoint2[index]["y"]]
        else:
            return self.waypoint[index]
    
    def get_waypoint_all(self):
        if self.ver_waypoint2:
            return self.waypoint2
        else:
            return self.waypoint
    
    def get_waypoint_label(self,index):
        if self.ver_waypoint2:
            return self.waypoint2[index]["label"]
    


    def on_enter_follow_path(self):
        print("callback on_enter_follow_path")
        self.waypoint_target = self.waypoint[1]
        self.waypoint_index = 1
        # self.state = 'follow_path'

    def set_checkpoint(self,checkpoint):
        self.checkpoint = checkpoint
    def get_checkpoint(self):
        return self.checkpoint

    def update(self):

        print("1. update")
        # Check state
        if (self.state == 'follow_path'):
            
            print(f"waypoint length: {len(self.waypoint)}")
            a = self.waypoint_target[0]-self.position[0]    #x
            b = self.waypoint_target[1]-self.position[1]    #y
            radius = math.sqrt(math.pow(a,2)+math.pow(b,2))
            print(f"radius to wp: {radius:.2f}")
            if (radius < self.radius_wp): # reach position? compare with set radius tolerance
                #change waypoint
                print(f"wp_index: {self.waypoint_index} length of waypoint: {len(self.waypoint)}")
                if(self.waypoint_index < len(self.waypoint)):  #still another waypoint to go
                    self.waypoint_index += 1
                    if(self.waypoint_index > len(self.waypoint)-1): #exceed number of waypoint
                        SM_Car.drive(self,0,0,0)
                        self.trigger('finish')
                        print("finish")
                    else:
                        self.waypoint_target = self.waypoint[self.waypoint_index]
                        print("move to next waypoint")
            else: #move

                # get angle of waypoint
                wp_angle = math.degrees(math.atan2(b,a))
                wp_angle %= 360                
                
                # print(f"distance to waypoint: {math.sqrt(math.pow(a,2)+math.pow(b,2)):.2f}")
                #print(f"waypoint target: {self.waypoint_target}")
                
                # print(f"angle to waypoint: {wp_angle:.2f}")
                # print(f"car_angle: {self.car_angle:.2f}")
                dif_angle = wp_angle - self.car_angle
                # print(f"dif_angle: {dif_angle:.2f}")
                
                #check car angle
                if(self.car_angle >= 0 and self.car_angle < 90):    #Q1
                    if(wp_angle >= self.car_angle and wp_angle <= self.car_angle + 180):
                        self.turnRight(dif_angle)
                        # print("Q1 right")
                    elif (wp_angle < self.car_angle or wp_angle > 180 + self.car_angle):
                        self.turnLeft(dif_angle)
                        # print("Q1 left")
                    else:
                        print("unknown_1")
                elif(self.car_angle >= 90 and self.car_angle < 180):    #Q2
                    if(wp_angle >= self.car_angle and wp_angle <= self.car_angle + 180):
                        self.turnRight(dif_angle)
                        # print("Q2 right")
                    elif (wp_angle < self.car_angle or wp_angle > 180 + self.car_angle):
                        self.turnLeft(dif_angle)
                        # print("Q2 left")
                    else:
                        print("unknown_2")
                elif(self.car_angle >= 180 and self.car_angle < 270):    #Q3
                    if(wp_angle >= self.car_angle or wp_angle <= self.car_angle-180):
                        self.turnRight(dif_angle)
                        # print("Q3 right")
                    elif (wp_angle < self.car_angle and wp_angle > self.car_angle-180):
                        self.turnLeft(dif_angle)
                        print("Q3 left")
                    else:
                        print("unknown_3")
                elif(self.car_angle >= 270 and self.car_angle < 360):    #Q4
                    if(wp_angle >= self.car_angle or wp_angle <= self.car_angle-180):
                        self.turnRight(dif_angle)
                        # print("Q4 right")
                    elif (wp_angle < self.car_angle and wp_angle > self.car_angle-180):
                        self.turnLeft(dif_angle)
                        # print("Q4 left")
                    else:
                        print("unknown_4")
                else:
                    print("unknown_5")
                

                SM_Car.drive(self,2,self.steer_angle,0) #set drive
                # print(f"steer_angle: {self.steer_angle:.2f}")
            # print(f"waypoint index: {self.waypoint_index}")

        elif (self.state == 'idle'):
            SM_Car.drive(self,0,0,70) #set drive

        elif (self.state == 'move'):
            print('move')
            
            # if self.get_checkpoint == 0:
            #     #get road profile according to check point

            #     a = self.waypoint_target[0]-self.position[0]    #x
            #     b = self.waypoint_target[1]-self.position[1]    #y
            #     radius = math.sqrt(math.pow(a,2)+math.pow(b,2))
            # if (radius < self.radius_wp): # reach position? compare with set radius tolerance
            #     #change waypoint
            #     if(self.waypoint_index < len(self.waypoint)):  #still another waypoint to go
            #         self.waypoint_index += 1
            #         if(self.waypoint_index > len(self.waypoint)-1): #exceed number of waypoint
            #             SM_Car.drive(self,0,0,0)
            #             self.trigger('finish')
            #         else:
            #             self.waypoint_target = self.waypoint[self.waypoint_index]
            # else: #move

            #     # get angle of waypoint
            #     wp_angle = math.degrees(math.atan2(b,a))
            #     wp_angle %= 360                
                
            #     # print(f"distance to waypoint: {math.sqrt(math.pow(a,2)+math.pow(b,2)):.2f}")
            #     #print(f"waypoint target: {self.waypoint_target}")
                
            #     # print(f"angle to waypoint: {wp_angle:.2f}")
            #     # print(f"car_angle: {self.car_angle:.2f}")
            #     dif_angle = wp_angle - self.car_angle
            #     # print(f"dif_angle: {dif_angle:.2f}")
                
            #     #check car angle
            #     if(self.car_angle >= 0 and self.car_angle < 90):    #Q1
            #         if(wp_angle >= self.car_angle and wp_angle <= self.car_angle + 180):
            #             self.turnRight(dif_angle)
            #             # print("Q1 right")
            #         elif (wp_angle < self.car_angle or wp_angle > 180 + self.car_angle):
            #             self.turnLeft(dif_angle)
            #             # print("Q1 left")
            #         else:
            #             # print("unknown_1")
            #     elif(self.car_angle >= 90 and self.car_angle < 180):    #Q2
            #         if(wp_angle >= self.car_angle and wp_angle <= self.car_angle + 180):
            #             self.turnRight(dif_angle)
            #             # print("Q2 right")
            #         elif (wp_angle < self.car_angle or wp_angle > 180 + self.car_angle):
            #             self.turnLeft(dif_angle)
            #             # print("Q2 left")
            #         else:
            #             # print("unknown_2")
            #     elif(self.car_angle >= 180 and self.car_angle < 270):    #Q3
            #         if(wp_angle >= self.car_angle or wp_angle <= self.car_angle-180):
            #             self.turnRight(dif_angle)
            #             # print("Q3 right")
            #         elif (wp_angle < self.car_angle and wp_angle > self.car_angle-180):
            #             self.turnLeft(dif_angle)
            #             # print("Q3 left")
            #         else:
            #             # print("unknown_3")
            #     elif(self.car_angle >= 270 and self.car_angle < 360):    #Q4
            #         if(wp_angle >= self.car_angle or wp_angle <= self.car_angle-180):
            #             self.turnRight(dif_angle)
            #             # print("Q4 right")
            #         elif (wp_angle < self.car_angle and wp_angle > self.car_angle-180):
            #             self.turnLeft(dif_angle)
            #             # print("Q4 left")
            #         else:
            #             # print("unknown_4")
            #     else:
            #         # print("unknown_5")
                

            #     SM_Car.drive(self,2,self.steer_angle,0) #set drive


        #################   CAR DYNAMIC SECTION ####################
        #calculate internal dynamic (from car acceleration to force) 
        Car.calcCarAccelerationMag(self)
        Car.calcCarAccelerationVec(self)
        Car.calcInternalForce(self) #including brake force
        Car.calcCentripetal(self)

        #calculate external (drag, etc)
        Car.calcForceDrag(self)
        Car.calcForceResultant(self)
        Car.calcAccelerationResult(self)

        # forward straight motion
        if (abs(self.steer_angle) < 0.1): #forward straight motion
            self.velocity = np.add(self.velocity,self.acceleration_r)
            self.position = np.add(self.position,self.velocity)
            self.radius_c.fill(0)

        # TURN
        else:
            vpa = np.add(self.velocity, self.acceleration_r)
            speed = np.linalg.norm(vpa)
            self.velocity[0] = speed*math.cos(math.radians(self.car_angle))
            self.velocity[1] = speed*math.sin(math.radians(self.car_angle))
            self.position = np.add(self.position, self.velocity)

            #heading change
            self.car_angle += math.degrees((speed/self.length)*math.tan(math.radians(self.steer_angle)))
            self.car_angle %= 360

        # print(f"position: {self.position}")
        # print(f"velocity: {self.velocity}")
        # print(f"acceleration: {self.acceleration_r}")
        # print(f"car_angle: {self.car_angle:.2f}")
        print(f"OVERALL: force: {np.linalg.norm(self.force_result_v):.2f} | acceleration: {np.linalg.norm(self.acceleration_r):.2f}")
        print(f"| velocity: {np.linalg.norm(self.velocity):.2f} | position: {self.position} | car angle: {self.car_angle:.2f}")
    
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

    #################### draw    #################

    def draw(self, win):
        # Draw the car as a rotated rectangle
        car_surface = pygame.Surface((self.width, self.length), pygame.SRCALPHA)
        car_surface.fill((0, 255, 0))  # Green color
        scaled_image = pygame.transform.scale(self.image, (50, 24))

        if self.init_set:   # at initial start the car needs to generate rotated_car variable
            rotated_image = pygame.transform.rotate(scaled_image, -self.car_angle)
            self.init_set = False
        elif abs(np.linalg.norm(self.velocity) > 0.001):
            rotated_image = pygame.transform.rotate(scaled_image, -self.car_angle)
        else:
            rotated_image = pygame.transform.rotate(scaled_image, -self.car_angle)
        car_rect = rotated_image.get_rect(center=(self.position[0], self.position[1]))

        win.blit(rotated_image, car_rect)

        if self.show_state:
            font = pygame.font.SysFont("Arial", 14)
            text_surface = font.render(self.state, True, (0, 0, 0))
            win.blit(text_surface, (self.position[0], self.position[1]))
    
