""" generate series of waypoint path """
import pygame
import math
import numpy as np

class Path:

    def __init__(self,waypoint, screen_width, screen_height):
        self.waypoint = waypoint
        self.width = screen_width
        self.height = screen_height

    def draw(self, win):
        line = pygame.draw.lines(win,[0,0,0],False,self.waypoint,1)