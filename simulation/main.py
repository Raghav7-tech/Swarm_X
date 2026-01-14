import pygame
import sys
import numpy as np
from settings import *
from map_manager import generate_city
from pathfinder import Pathfinder
from drone import Drone

def main(): 
    pygame.init()  
    info = pygame.display.Info()
    SCREEN_W, SCREEN_H = info.current_w - 50, info.current_h - 50
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("SWARM_X | Advanced Edition")
    clock = pygame.time.Clock() 
    pathfinder = Pathfinder(SCREEN_W, SCREEN_H, CELL_SIZE) 
    buildings = generate_city(pathfinder, SCREEN_W, SCREEN_H) 
    drones = []
    selected_drone = None
    spawn_mode = PRIORITY_COM  
    
    print("\n--- CONTROLS ---")
    print("[SPACE] : Spawn Drone")
    print("[L-CLICK]: Select Drone")
    print("[R-CLICK]: Set Path (Drone waits in READY)")
    print("[ENTER] : LAUNCH ALL READY DRONES")
    print("[1, 2, 3]: Switch Drone Type")
    print("[R]     : Regenerate City")
    print("----------------\n")

    running = True
    while running: 
        for event in pygame.event.get():
            if event.type == pygame.QUIT: 
                running = False 
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos() 
                 
                if event.button == 1:
                    clicked_something = False
                    for d in drones: 
                        if np.linalg.norm(d.pos - np.array([mx, my])) < 20: 
                            if selected_drone: selected_drone.selected = False
                            selected_drone = d
                            d.selected = True
                            clicked_something = True
                            break
                    if not clicked_something and selected_drone:
                        selected_drone.selected = False
                        selected_drone = None 
                
                # RIGHT CLICK: Plan Path
                elif event.button == 3 and selected_drone:
                    path = pathfinder.search(selected_drone.pos, (mx, my)) 
                    if path: 
                        selected_drone.set_plan(path, (mx, my))
                        print(f"Plan set for {selected_drone.id_str}. Press ENTER.")
             
            if event.type == pygame.KEYDOWN: 
                if event.key == pygame.K_RETURN:
                    for d in drones: d.launch()

                if event.key == pygame.K_SPACE:
                    mx, my = pygame.mouse.get_pos() 
                    safe = True
                    hitbox = pygame.Rect(mx-5, my-5, 10, 10)
                    for b in buildings:
                        if b.rect.colliderect(hitbox): 
                            safe = False
                            break
                    if safe:
                        drones.append(Drone(mx, my, len(drones)+1, spawn_mode))
                 
                if event.key == pygame.K_1: spawn_mode = PRIORITY_COM
                if event.key == pygame.K_2: spawn_mode = PRIORITY_MIL
                if event.key == pygame.K_3: spawn_mode = PRIORITY_MED
                if event.key == pygame.K_r: 
                    drones.clear(); selected_drone = None
                    pathfinder = Pathfinder(SCREEN_W, SCREEN_H, CELL_SIZE)
                    buildings = generate_city(pathfinder, SCREEN_W, SCREEN_H)

        for d in drones:
            d.update(drones, buildings, pathfinder)
 
        screen.fill(BG_COLOR)
        for b in buildings: b.draw(screen)
          
        for d in drones:
            if d.status == "LANDED" or not d.alive: d.draw(screen)
        for d in drones:
            if d.status != "LANDED" and d.alive: d.draw(screen)
  
        pygame.draw.rect(screen, (10, 20, 10), (0, SCREEN_H-35, SCREEN_W, 35))
        font = pygame.font.SysFont("Arial", 16, bold=True)
        
        mode_str, mode_col = "COMMERCIAL", COLOR_COMMERCIAL
        if spawn_mode == PRIORITY_MIL: mode_str, mode_col = "MILITARY", COLOR_MILITARY
        elif spawn_mode == PRIORITY_MED: mode_str, mode_col = "MEDICAL", COLOR_MEDICAL

        txt_info = font.render(f"UNITS: {len(drones)} | [ENTER] LAUNCH | MODE:", True, (200, 200, 200))
        txt_mode = font.render(f" [{mode_str}]", True, mode_col)
        
        screen.blit(txt_info, (20, SCREEN_H - 28))
        screen.blit(txt_mode, (350, SCREEN_H - 28))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()