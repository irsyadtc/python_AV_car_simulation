import pygame
import math
import numpy as np

class Car:
    def __init__(self, x, y, rad):
        self.x = x
        self.y = y
        self.car_angle = rad  # Facing right initially
        self.speed = 0
        self.rotation_speed = 2
        self.steer_angle = 0
        self.brake_percent = 0
        
        #force
        self.force_motor = np.array([0.0,0.0,0.0])
        self.force_centri = np.array([0.0,0.0,0.0])
        self.force_drag = np.array([0.0,0.0,0.0])
        self.force_result = np.array([0.0,0.0,0.0])
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
        self.max_speed = 2
        self.max_acceleration = 0.5
        self.max_steer = 45
        # DESIRED
        self.speed_desi = 0
        self.steering_angle_desi = 0

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
        self.brake_percent = brk    #brake

        print(f"1. drive: steer angle: {self.steer_angle}")


    def update(self):
        # Update position based on speed and car angle
        self.position_prev = self.position
        print("2. update")
        #steering
        # self.steer_angle = self.steering_angle_desi

        #acceleration
        Car.calcAccelerationMag(self)
        Car.calcAcceleration(self)
        Car.calcForce(self)
        Car.calcCentripetal(self)
        Car.calcForceResultant(self)
        Car.calcAccelerationResult(self)

        # forward straight motion
        if (abs(self.steer_angle) < 0.1): #forward straight motion
            self.velocity = np.add(self.velocity,self.acceleration_r)
            self.position = np.add(self.position,self.velocity)
            self.radius_c.fill(0)
            self.center_c.fill(0)

        # TURN
        # elif (self.steer_angle > 0 or self.steer_angle < 0):
        else:
            # Car.calcRadiusTurn(self)
            # Car.calcCenterTurn(self)
            
            vpa = np.add(self.velocity, self.acceleration_r)
            print(f"vpa: {vpa}")
            speed = np.linalg.norm(vpa)
            print(f"speed: {speed}")

            print(f"cos({math.radians(self.car_angle):.2f}) = {math.cos(math.radians(self.car_angle))}")
            self.velocity[0] = speed*math.cos(math.radians(self.car_angle))
            print(f"velocity[0]: {self.velocity[0]}")
            self.velocity[1] = speed*math.sin(math.radians(self.car_angle))
            print(f"velocity: {self.velocity}")

            # self.velocity = np.add(self.velocity, self.acceleration_r)

            self.position = np.add(self.position, self.velocity)

            #heading change
            self.car_angle += math.degrees((speed/self.length)*math.tan(math.radians(self.steer_angle)))

        print(f"position: {self.position}")
        print(f"velocity: {self.velocity}")
        print(f"acceleration: {self.acceleration_r}")
        print(f"car_angle: {self.car_angle:.2f}")

        # car_rad = math.radians(self.car_angle)
        # steer_rad = math.radians(self.steer_angle)
        # r_icc = self.length/math.tan(steer_rad)

        # print(f"car_angle: {self.car_angle} {car_rad:.2f}")
        # print(f"steer_angle: {self.steer_angle} {steer_rad:.2f}") 
        # print(f"r_icc: {r_icc:.2f}")

        # dx = self.speed * math.cos(r_icc)
        # dy = self.speed * math.sin(r_icc)
        # dx = self.speed * math.sin(r_icc)
        # dy = self.speed * math.cos(r_icc)
        
        # self.x += dx
        # self.y += dy
        # self.car_angle += self.steer_angle
        #test
        # self.car_angle -= 1

        # Keep angle between 0 and 360 degrees
        # self.car_angle %= 360

        # print("car.update: speed: ", self.speed,"\n car.update: car_angle: ", self.car_angle)


    #####################################################
    def calcAccelerationMag(self):
        error = self.speed_desi - np.linalg.norm(self.velocity)
        if (error < 0.01): #now achieve that speed
            print("2.1. cAM: achieve speed")
            self.acceleration = 0
        elif (self.max_acceleration -  self.acceleration <= 0):
            self.acceleration = self.max_acceleration
            print("2.1. cAM: max_acceleration")
        else:
            self.acceleration += 0.01
            print(f"2.1. cAM: increase acceleration")
        print(f"error speed_desired-current_speed: {error:.2f}")
        print(f"acceleration: {self.acceleration:.2f}")

    def calcAcceleration(self):
        self.acceleration_m[0] = math.cos(math.radians(self.car_angle))
        self.acceleration_m[1] = math.sin(math.radians(self.car_angle))
        if (self.acceleration < 0.001): # too small assume zero
            self.acceleration_m = np.zeros(3)
        else:
            np.multiply(self.acceleration_m,self.acceleration)
        print(f"2.2. calcAcceleration: acceleration_m: {self.acceleration_m}")

    def calcForce(self):
        self.force_motor[0] = self.acceleration_m[0]*self.mass
        self.force_motor[1] = self.acceleration_m[1]*self.mass
        print(f"2.3. calcForce: force_motor: {self.force_motor}")

    def calcCentripetal(self):
        if (np.linalg.norm(self.radius_c) < 0.001):
            self.force_centri = [0,0,0]
        else:
            self.force_centri = self.radius_c
            self.force_centri = Car.rotate(self.force_centri,math.pi)
            m = ((self.mass*math.pow(np.linalg.norm(self.velocity),2))/np.linalg.norm(self.radius_c))
            self.force_centri = np.dot(self.force_centri,m)
        print(f"2.4. cC: force_centri: {self.force_centri}")

    def calcForceResultant(self):
        self.force_result[0] = self.force_motor[0] + self.force_drag[0] + self.force_centri[0]
        self.force_result[1] = self.force_motor[1] + self.force_drag[1] + self.force_centri[1]
        print(f"2.5. cFR: acceleration_m: {self.acceleration_m}")
    
    def calcAccelerationResult(self):
        self.acceleration_r = self.force_result/self.mass
        print(f"2.6. cAR: acceleration_r: {self.acceleration_r}")

    # Calculate the radius vector of ICC 
    def calcRadiusTurn(self):
        h = np.zeros(3)
        mag = 0

        if (self.steer_angle < 0): #left turn
            print("2.7. cRT: left turn")
            h[0] = math.cos(self.car_angle)
            h[1] = math.sin(self.car_angle)
            h = Car.rotate(h,math.pi/2)
            mag = self.length*math.tan((math.pi/2)-(-1*math.radians(self.steer_angle)))
            self.radius_c[0] = mag*h[0]
            self.radius_c[1] = mag*h[1]

        elif (self.steer_angle > 0): #right turn
            print("2.7. cRT: right turn")
            h[0] = math.cos(self.car_angle)
            h[1] = math.sin(self.car_angle)
            h = Car.rotate(h,-math.pi/2)
            mag = self.length*math.tan((math.pi/2)-(math.radians(self.steer_angle)))
            self.radius_c[0] = mag*h[0]
            self.radius_c[1] = mag*h[1]
        
        print(f"cRT radius_c: {self.radius_c}")

    # Rotate vector
    def rotate(vector,add_rad):
        mag = math.hypot(vector[0],vector[1])
        rad = math.atan2(vector[1],vector[0])
        vector[0] = mag*math.cos(rad+add_rad)
        vector[1] = mag*math.sin(rad+add_rad)
        return vector
    
    def calcCenterTurn(self):
        self.center_c[0] = self.position[0] - self.radius_c[0]
        self.center_c[1] = self.position[1] - self.radius_c[1]
        print(f"2.8. cCT center_c: {self.center_c}")
    
    ###################################################

    def draw(self, win):
        # Draw the car as a rotated rectangle
        car_surface = pygame.Surface((self.width, self.length), pygame.SRCALPHA)
        car_surface.fill((0, 255, 0))  # Green color
        scaled_image = pygame.transform.scale(self.image, (50, 24))
        # print("car.draw: self.speed: ", self.speed
            #   ,"\ncar.draw: self.car_angle: ", self.car_angle)

        if self.init_set:   # at initial start the car needs to generate rotated_car variable
            # rotated_car = pygame.transform.rotate(car_surface, -self.car_angle)
            rotated_image = pygame.transform.rotate(scaled_image, -self.car_angle)
            self.init_set = False
            print("init rotate")
        # elif abs(self.velocity) > 0.001:   # if the car moves else car remain unturn
        elif abs(np.linalg.norm(self.velocity) > 0.001):
            # rotated_car = pygame.transform.rotate(car_surface, -self.car_angle)
            rotated_image = pygame.transform.rotate(scaled_image, -self.car_angle)
            print("car moves")
        else:
            # rotated_car = pygame.transform.rotate(car_surface, -self.car_angle)
            rotated_image = pygame.transform.rotate(scaled_image, -self.car_angle)
            print("car idle" \
            "car_angle:", self.car_angle)
        # car_rect = rotated_car.get_rect(center=(self.x, self.y))
        car_rect = rotated_image.get_rect(center=(self.position[0], self.position[1]))


        # scaled_image = pygame.transform.scale(self.image, (50, 24))
        # rotated_image = pygame.transform.rotate(scaled_image, 0)
        # rotated_image = pygame.transform.rotate(self.image, 45)

        # win.blit(rotated_image, self.x, self.y)
        win.blit(rotated_image, car_rect)
        # win.blit(rotated_car, car_rect)



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