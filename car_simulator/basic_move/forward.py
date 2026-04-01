import sys
import os
import pygame
import math

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),"..")))

# Now we can import our car file
try:
    import car
    print("Successfully connected to the project configuration.")
except ImportError as e:
    print(f"Connection failed: {e}")

pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Self-Driving Car Simulation")



def main():
    print("Basic Forward ")
    car_ = car.Car(160, 200, 0)   #x,y,angle
    clock = pygame.time.Clock()
    start_tick = pygame.time.get_ticks()
    run = True

    while run:
        clock.tick(40)
        win.fill((173, 173, 173))   #color

        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        
        # make_decision(car_)
        # steering angle >0 steer right, <0 steer left
        car_.drive(2,-45,0) #(speed, steering angle, brake)
        car_.update()
        car_.draw(win)

        current_time = pygame.time.get_ticks()
        dt = (current_time - start_tick)/ 1000
        # print("ticks: ", dt)
        # print(type(dt))
        # text = f"ticks: {dt}"
        font = pygame.font.SysFont("Arial", 14)
        text_surface = font.render(f"time: {dt:.1f}", True, (0, 0, 0))
        win.blit(text_surface, (40, 40))


        pygame.display.update()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()