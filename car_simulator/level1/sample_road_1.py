""" example of road generation """
import sys
import os
import pygame
import math



sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),"..")))

# Now we can import our car file
try:
    import car
    import road
    print("Successfully connected to the project configuration.")
except ImportError as e:
    print(f"Connection failed: {e}")

pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Self-Driving Car Simulation: Road Generation")


def main():
    #instant
    car1_ = car.Car(40,300,0)
    car2_ = car.Car(100,300,0)
    car2_.set_car_image('car_sedan_blue.png')
    car3_ = car.Car(160,300,0)
    car3_.set_car_image('car_sedan_red.png')
    car4_ = car.Car(220,300,0)
    car4_.set_car_image('car_sedan_yellow.png')

    car5_ = car.Car(700,335,180)
    car6_ = car.Car(500,335,180)
    car6_.set_car_image('car_sedan_blue.png')
    car7_ = car.Car(611,335,180)
    car7_.set_car_image('car_sedan_red.png')
    car8_ = car.Car(400,300,90)
    car8_.set_car_image('car_sedan_yellow.png')

    car9_ = car.Car(400,220,90)
    car10_ = car.Car(365,400,270)
    car10_.set_car_image('car_sedan_blue.png')
    car11_ = car.Car(365,550,270)
    car11_.set_car_image('car_sedan_red.png')
    car12_ = car.Car(600,300,0)
    car12_.set_car_image('car_sedan_yellow.png')


    RWp1_ = [(0,300) ,(800,300)]
    Road1 = road.Road('Jalan Aman',1,0,RWp1_)
    RWp2_ = [(400,0),(400,600)]
    Road2 = road.Road('Jalan Maju',1,1,RWp2_)
    RWp3_ = [(0,335) ,(800,335)]
    Road3 = road.Road('Jalan Aman',1,0,RWp3_)
    RWp4_ = [(365,0),(365,600)]
    Road4 = road.Road('Jalan Maju',1,1,RWp4_)

    #clock
    clock = pygame.time.Clock()
    start_tick = pygame.time.get_ticks()
    run = True

    #game control
    pause = False

    while run:
        for e in pygame.event.get():
            if e.type == pygame.QUIT: break
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_SPACE: pause = not pause
        if not pause:
            clock.tick(40)
            win.fill((134, 207, 122))   #grass color

            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = False

            # steering angle >0 steer right, <0 steer left
            car1_.drive(0,0,100)
            car2_.drive(0,0,100)
            car3_.drive(0,0,100)
            car4_.drive(0,0,100)
   
            car5_.drive(0,0,100)
            car6_.drive(0,0,100)
            car7_.drive(0,0,100)
            car8_.drive(0,0,100)

            car9_.drive(0,0,100)
            car10_.drive(0,0,100)
            car11_.drive(0,0,100)
            car12_.drive(0,0,100)

            #road
            Road1.draw(win)
            Road2.draw(win)
            Road3.draw(win)
            Road4.draw(win)
            #car draw
            car1_.draw(win)
            car2_.draw(win)
            car3_.draw(win)
            car4_.draw(win)

            car5_.draw(win)
            car6_.draw(win)
            car7_.draw(win)
            car8_.draw(win)

            car9_.draw(win)
            car10_.draw(win)
            car11_.draw(win)
            car12_.draw(win)

            #time counter
            current_time = pygame.time.get_ticks()
            dt = (current_time - start_tick)/ 1000
            font = pygame.font.SysFont("Arial", 14)
            text_surface = font.render(f"time: {dt:.1f}", True, (0, 0, 0))
            win.blit(text_surface, (40, 40))


            pygame.display.update()
        else:
            pass

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()