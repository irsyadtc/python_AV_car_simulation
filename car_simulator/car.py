import pygame
import math
import numpy as np

class Car:
    def __init__(self, x, y, rad):
        self.car_angle = rad  # Facing right initially
        self.speed = 0
        self.rotation_speed = 2
        self.steer_angle = 0
        self.brake_input = 0 #in percentage
        self.force_brake_velocity_control = 0   #in percentage
        
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
        self.position = np.array([x,y,0])
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
        #collision sensing
        self.sensors = []
        self.sensor_range = 100
        self.radius = max(self.length, self.width) / 2  # For collision detection
        #car image
        self.image = pygame.image.load('car_sedan_2.png') #original size 768x361

        self.init_set = True

        self.col_pts_ag3 = []
        self.col_pts_ag4 = []

    def drive(self, spd, steer, brk):
        if (spd > self.max_speed):  #speed
            self.speed_desi = self.max_speed
        else:
            self.speed_desi = spd
        if (abs(steer) > self.max_steer):   #steer
            self.steer_angle = self.max_steer*(steer/abs(steer))
        else:
            self.steer_angle = steer
        self.brake_input = brk    #brake
        # print(f"1. drive: speed desi: {self.speed_desi}")
        # print(f"1. drive: steer angle: {self.steer_angle}")


    def update(self):
        # Update position based on speed and car angle
        self.position_prev = self.position
        # print("2. update")

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
            self.center_c.fill(0)

        # TURN
        else:

            vpa = np.add(self.velocity, self.acceleration_r)
            # print(f"vpa: {vpa}")
            speed = np.linalg.norm(vpa)
            # print(f"speed: {speed}")

            # print(f"cos({math.radians(self.car_angle):.2f}) = {math.cos(math.radians(self.car_angle))}")
            self.velocity[0] = speed*math.cos(math.radians(self.car_angle))
            # print(f"velocity[0]: {self.velocity[0]}")
            self.velocity[1] = speed*math.sin(math.radians(self.car_angle))
            # print(f"velocity: {self.velocity}")

            # self.velocity = np.add(self.velocity, self.acceleration_r)

            self.position = np.add(self.position, self.velocity)

            #heading change
            self.car_angle += math.degrees((speed/self.length)*math.tan(math.radians(self.steer_angle)))

        # print(f"position: {self.position}")
        # print(f"velocity: {self.velocity} mag: {np.linalg.norm(self.velocity):.2f}")
        # print(f"acceleration: {self.acceleration_r}")
        # print(f"car_angle: {self.car_angle:.2f}")
        print(f"OVERALL: force: {np.linalg.norm(self.force_result_v):.2f} | acceleration: {np.linalg.norm(self.acceleration_r):.2f}")
        print(f"| velocity: {np.linalg.norm(self.velocity):.2f} | position: {self.position} | car angle: {self.car_angle:.2f}")


    #####################################################
    def calcCarAccelerationMag(self):
        error = self.speed_desi - np.linalg.norm(self.velocity)
        if (error < 0.00001 and error > -0.00001): #reach the speed
            # print("2.1. cAM: achieve speed")
            self.acceleration = 0
            self.force_brake_velocity_control = 0
        elif (error < -0.00001): #exceed speed
            self.acceleration = 0
            self.force_brake_velocity_control = 10
        else:
            self.acceleration = self.const_acceleration
            self.force_brake_velocity_control = 0
        # print(f"error speed_desired-current_speed: {error:.2f}")
        # print(f"acceleration: {self.acceleration:.2f}")

    def calcCarAccelerationVec(self):
        self.acceleration_m[0] = self.acceleration*math.cos(math.radians(self.car_angle))
        self.acceleration_m[1] = self.acceleration*math.sin(math.radians(self.car_angle))
        # if (self.acceleration < 0.001): # too small assume zero
        #     self.acceleration_m = np.zeros(3)
        # else:
        #     np.multiply(self.acceleration_m,self.acceleration)
        # np.multiply(self.acceleration_m,self.acceleration)
        # print(f"2.2. calcCarAccelerationVec: acceleration_m: {self.acceleration_m}")

    def calcInternalForce(self):
        Car.calcMotorForce(self)
        Car.calcBrakeForce(self)
        self.force_car_v = self.force_motor_v + self.force_brake_v
        # print(f"2.3. calcInternalForce: force_car_v: {self.force_car_v}")

    def calcBrakeForce(self):
        from_drive_input_brake = (self.brake_input/100)*self.brake_coef*np.linalg.norm(self.velocity)
        from_velocity_control = (self.force_brake_velocity_control/100)*self.brake_coef*np.linalg.norm(self.velocity)
        self.force_brake = from_drive_input_brake + from_velocity_control

        unit_vector = np.array([0.0,0.0])
        mag = np.linalg.norm(self.velocity)
        if(mag <= 0 or np.isnan(mag)):
            unit_vector = [0.0,0.0]
        else:
            unit_vector = self.velocity/mag
        self.force_brake_v[0] = -1*self.force_brake*unit_vector[0]  #minus one since it is opposing the car direction
        self.force_brake_v[1] = -1*self.force_brake*unit_vector[1]

    def calcMotorForce(self):
        self.force_motor_v[0] = (self.acceleration_m[0]*self.mass)
        self.force_motor_v[1] = (self.acceleration_m[1]*self.mass)

    #to be revised
    def calcCentripetal(self):
        if (np.linalg.norm(self.radius_c) < 0.001):
            self.force_centri = [0,0,0]
        else:
            self.force_centri = self.radius_c
            self.force_centri = Car.rotate(self.force_centri,math.pi)
            m = ((self.mass*math.pow(np.linalg.norm(self.velocity),2))/np.linalg.norm(self.radius_c))
            self.force_centri = np.dot(self.force_centri,m)
            # print(f"2.4. cC: force_centri: {self.force_centri}")

    def calcForceDrag(self):
        #assume to be opposite of the motor force direction
        #assume 1/100 of forward force
        mag = np.linalg.norm(self.force_car_v)
        self.force_drag[0] = (-1*self.force_car_v[0])*self.const_drag
        self.force_drag[1] = (-1*self.force_car_v[1])*self.const_drag
        # print(f"2.5. cFD: frc_drg: {self.force_drag}")

    def calcForceResultant(self):
        self.force_result_v[0] = self.force_car_v[0] + self.force_drag[0] + self.force_centri[0]
        self.force_result_v[1] = self.force_car_v[1] + self.force_drag[1] + self.force_centri[1]
        # print(f"2.5. cFR: frc_rslt: {self.force_result_v}")
    
    def calcAccelerationResult(self):
        self.acceleration_r = self.force_result_v/self.mass
        # print(f"2.6. cAR: acceleration_r: {self.acceleration_r}")

    # Rotate vector
    def rotate(vector,add_rad):
        mag = math.hypot(vector[0],vector[1])
        rad = math.atan2(vector[1],vector[0])
        vector[0] = mag*math.cos(rad+add_rad)
        vector[1] = mag*math.sin(rad+add_rad)
        return vector
    
    
    ###################################################

    def draw(self, win):
        # Draw the car as a rotated rectangle
        car_surface = pygame.Surface((self.width, self.length), pygame.SRCALPHA)
        car_surface.fill((0, 255, 0))  # Green color
        scaled_image = pygame.transform.scale(self.image, (50, 24))
        # print("car.draw: self.speed: ", self.speed
            #   ,"\ncar.draw: self.car_angle: ", self.car_angle)

        if self.init_set:   # at initial start the car needs to generate rotated_car variable
            rotated_image = pygame.transform.rotate(scaled_image, -self.car_angle)
            self.init_set = False
            # print("init rotate")
        elif abs(np.linalg.norm(self.velocity) > 0.001):
            rotated_image = pygame.transform.rotate(scaled_image, -self.car_angle)
            # print("car moves")
        else:
            rotated_image = pygame.transform.rotate(scaled_image, -self.car_angle)
            # print("car idle" \
            # "car_angle:", self.car_angle)
        car_rect = rotated_image.get_rect(center=(self.position[0], self.position[1]))

        win.blit(rotated_image, car_rect)

    #########################################################################

    def set_car_image(self, img):
        self.image = pygame.image.load(img)
        
    #########################################################################

    def cast_sensors(self, walls):
        self.sensors = []
        sensor_angles = [-90, -60, -30, 0, 30, 60, 90]  # More sensors for better detection
        for angle_offset in sensor_angles:
            angle = self.car_angle + angle_offset
            rad = math.radians(angle)
            end_x = self.x + self.sensor_range * math.cos(rad)
            end_y = self.y + self.sensor_range * math.sin(rad)
            sensor_line = ((self.x, self.y), (end_x, end_y))
            # Check for collision with walls
            collision_point = None
            min_distance = self.sensor_range
            for wall in walls:
                hit_point = line_rect_collision(sensor_line, wall)
                if hit_point:
                    distance = math.hypot(hit_point[0] - self.x, hit_point[1] - self.y)
                    if distance < min_distance:
                        min_distance = distance
                        collision_point = hit_point
            if collision_point:
                # pygame.draw.line(win, (255, 0, 0), (self.x, self.y), collision_point, 1)
                self.col_pts_ag3 = (self.x, self.y)
                self.col_pts_ag4 = collision_point
            else:
                # pygame.draw.line(win, (255, 0, 0), (self.x, self.y), (end_x, end_y), 1)
                self.col_pts_ag3 = (self.x, self.y)
                self.col_pts_ag4 = (end_x, end_y)
            self.sensors.append(min_distance)


def line_rect_collision(line, rect):
    x1, y1 = line[0]
    x2, y2 = line[1]
    # Get rect sides
    rect_lines = [
        ((rect.left, rect.top), (rect.right, rect.top)),
        ((rect.right, rect.top), (rect.right, rect.bottom)),
        ((rect.right, rect.bottom), (rect.left, rect.bottom)),
        ((rect.left, rect.bottom), (rect.left, rect.top))
    ]
    closest_point = None
    min_distance = float('inf')
    for rect_line in rect_lines:
        hit_point = line_line_collision(line, rect_line)
        if hit_point:
            distance = math.hypot(hit_point[0] - x1, hit_point[1] - y1)
            if distance < min_distance:
                min_distance = distance
                closest_point = hit_point
    return closest_point
    
def line_line_collision(line1, line2):
    # Line segments: line1 from (x1, y1) to (x2, y2), line2 from (x3, y3) to (x4, y4)
    x1, y1 = line1[0]
    x2, y2 = line1[1]
    x3, y3 = line2[0]
    x4, y4 = line2[1]

    denom = (y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1)
    if denom == 0:
        return None  # Lines are parallel

    ua = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / denom
    ub = -((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) / denom

    if 0 <= ua <= 1 and 0 <= ub <= 1:
    # Intersection point is within both line segments
        x = x1 + ua*(x2 - x1)
        y = y1 + ua*(y2 - y1)
        return (x, y)
    else:
        return None