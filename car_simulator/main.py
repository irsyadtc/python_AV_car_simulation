import pygame
import math
import sys

import car

pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Self-Driving Car Simulation")



def create_track():
    walls = []

    # Outer boundaries
    track_outer = [
        (100, 100, 600, 20),    # Top wall
        (680, 100, 20, 400),    # Right wall
        (100, 480, 600, 20),    # Bottom wall
        (100, 100, 20, 400)     # Left wall
    ]

    # Inner boundaries
    track_inner = [
        (200, 200, 400, 20),    # Top wall
        (580, 200, 20, 200),    # Right wall
        (200, 380, 400, 20),    # Bottom wall
        (200, 200, 20, 200)     # Left wall
    ]

    # Create walls for outer track
    for rect in track_outer:
        walls.append(pygame.Rect(rect))
   # Create walls for inner track
    for rect in track_inner:
        walls.append(pygame.Rect(rect))

    return walls

def draw_track(win, walls):
    for wall in walls:
        pygame.draw.rect(win, (0, 0, 0), wall)

def make_decision(car):
    sensors = car.sensors
    if sensors:
        front_sensor = sensors[3]
        left_sensors = sensors[:3]
        right_sensors = sensors[4:]

        # If an obstacle is detected in front, decide to turn
        if front_sensor < 30:
            if sum(left_sensors) > sum(right_sensors):
                car.angle -= car.rotation_speed  # Turn left
            else:
                car.angle += car.rotation_speed  # Turn right
            if car.speed > 0:
                car.speed -= car.acceleration  # Slow down
        else:
            # Adjust course slightly if side sensors detect walls
            if min(left_sensors) < 50:
                car.angle += car.rotation_speed / 2  # Adjust right
            elif min(right_sensors) < 50:
                car.angle -= car.rotation_speed / 2  # Adjust left
            if car.speed < car.max_speed:
                car.speed += car.acceleration  # Accelerate
    else:
        car.speed = 0  # Stop if no sensor data

def check_collision(car, walls):
    car_circle = pygame.Rect(
        car.x - car.radius, car.y - car.radius, car.radius*2, car.radius*2)
    for wall in walls:
        if car_circle.colliderect(wall):
            return True
    return False

def main():
    # Starting position adjusted to the track
    car_ = car.Car(160, 200, 90)   #x,y,angle
    walls = create_track()
    clock = pygame.time.Clock()
    run = True

    while run:
        clock.tick(60)
        win.fill((200, 200, 200))   #color

        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        # Update and draw track
        draw_track(win, walls)

        # Car operations
        # car_.cast_sensors(walls)
        #mod line
        # pygame.draw.line(win, (255, 0, 0), car_.col_pts_ag3, car_.col_pts_ag4, 1)

        # make_decision(car_)
        car_.update()

        # if check_collision(car_, walls):
        #     # If collision detected, stop the car and reverse a bit
        #     print("collision")  #debug
        #     car_.speed = -car_.max_speed / 2
        #     car_.update()
        #     car_.speed = 0

        car_.draw(win)

        pygame.display.update()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
