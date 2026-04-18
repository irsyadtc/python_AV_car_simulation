""" example of AV following pathway with state_machine control """
import sys
import os
import pygame
import math

from transitions import Machine
import stt_machine

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),"..")))

# Now we can import our car file
try:
    import car
    import path
    import sm_car
    print("Successfully connected to the project configuration.")
except ImportError as e:
    print(f"Connection failed: {e}")

pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Self-Driving Car Simulation")

# state machine
states = ['idle','follow_path','brake']
transitions = [
    {'trigger': 'start', 'source': 'idle', 'dest': 'follow_path'},
    {'trigger': 'pause', 'source': 'follow_path', 'dest': 'brake'},
    {'trigger': 'abort', 'source': '*', 'dest': 'idle'},
    {'trigger': 'finish', 'source': 'follow_path', 'dest': 'idle'}
    ]


def main():
    print("Path Following")
    # car_ = car.Car(200, 300, 0)   #x,y,angle

    car_init_position = (200,300,0)
    # wp1_ = [(car_.position[0],car_.position[1]) ,(600,300),(600,200),(500,100)]
    # sm_car_ = sm_car.SM_Car(car_init_position,states,transitions,'idle')
    sm_car_ = sm_car.SM_Car(car_init_position)
    # stt_machine_ = Machine(sm_car_,states=states,transitions=transitions,initial='idle')
    wp1_ = [(sm_car_.car.position[0],sm_car_.car.position[1]) ,(500,290),(600,350),(700,450)]

    sm_car_.set_waypoint(wp1_)
    #define callback for state machine to car
    # stt_machine.on_enter_follow_path('on_enter_follow_path_')

    clock = pygame.time.Clock()
    start_tick = pygame.time.get_ticks()
    run = True

    #game control
    pause = False


    sm_car_.trigger('start')

    while run:
        for e in pygame.event.get():
            if e.type == pygame.QUIT: break
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_SPACE: pause = not pause
        if not pause:
            clock.tick(40)
            win.fill((173, 173, 173))   #color

            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    run = False

            
            # make_decision(car_)
            # steering angle >0 steer right, <0 steer left
            # sm_car_.drive(0,0,0) #(speed, steering angle, brake)
            sm_car_.update()
            sm_car_.draw(win)

            wp1 = path.Path(wp1_,WIDTH,HEIGHT)
            wp1.draw(win)

            print(f"2. sm_car_state: {sm_car_.state}")

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