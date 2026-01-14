import pygame
import random
from settings import * 
class Building:
    def __init__(self, x, y, w, h): 
        self.rect=pygame.Rect(x,y,w,h)
         
    def draw(self, screen): 
        pygame.draw.rect(screen, BUILDING_FILL, self.rect)
        pygame.draw.rect(screen, BUILDING_OUTLINE, self.rect, 1)


def generate_city(pathfinder, width, height): 
    print("Generating City...")
    buildings = [] 
    MAIN_ROAD_WIDTH = 80
    BLOCK_SIZE = 250 
    for bx in  range(50, width - 50, BLOCK_SIZE + MAIN_ROAD_WIDTH):
        for by in range(50, height - 50, BLOCK_SIZE + MAIN_ROAD_WIDTH):
            current_x = bx
            while current_x < bx + BLOCK_SIZE:
                b_width = random.randint(40,100)
                if current_x + b_width > bx + BLOCK_SIZE:
                    b_width = (bx + BLOCK_SIZE) - current_x
                current_y = by
                while current_y < by + BLOCK_SIZE:
                    b_height = random.randint(40,100)
                    if current_y + b_height > by + BLOCK_SIZE:
                        b_height = (by + BLOCK_SIZE) - current_y
                    if random.random() > 0.1:
                        gap = random.randint(2,10)
                        if b_width > gap*2 and b_height > gap*2:
                            rect = pygame.Rect(current_x + gap, current_y + gap, b_width - gap*2, b_height - gap*2)
                            building = Building(rect.x, rect.y, rect.width, rect.height)
                            buildings.append(building)
                            pathfinder.add_obstacle(rect)
                    current_y += b_height
                current_x += b_width

            
    return buildings