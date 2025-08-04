import json
import time
import pygame
from math import log, ceil, sqrt
from visualisation import rnd_color
from geometry import Point

debug_pixels_per_meter = None
debug_reference_point = None
with open('src/config.json') as file:
    config_data = json.load(file)
    debug_pixels_per_meter = config_data['visualization']['pixels_per_meter']
    debug_grids_num_per_meter = config_data["environment"]["resolution"]
    debug_reference_point = (debug_pixels_per_meter / debug_grids_num_per_meter*2, debug_pixels_per_meter / debug_grids_num_per_meter*2)

color_dict = rnd_color(100)
def draw_circle(screen, color, center, radius = 5):
    try:
        pygame.draw.circle(screen, color, rectify_meter_to_pixel(center), radius, 0)
    except Exception as e:
        print "marrt*.py: unable to draw point {}, {}".format(center, e)

def rectify_meter_to_pixel(point):
    def meter_to_pixel(value):
        return int(ceil(value * debug_pixels_per_meter))
    return meter_to_pixel(point.x)+debug_reference_point[0],meter_to_pixel(point.y)+debug_reference_point[1]
