import pygame
import numpy as np
import random
from settings import *

class Drone:
    def __init__(self, x, y, uid_num, fixed_priority):
        self.id_num = uid_num   
        self.pos = np.array([float(x), float(y)]) 
        self.vel = np.array([0.0, 0.0]) 
        self.max_speed = MAX_SPEED    
        self.path = []          
        self.selected = False   
        self.status = "IDLE"    
        self.alive = True       
        self.prev_status = "IDLE" 
         
        self.priority = fixed_priority
        if self.priority == PRIORITY_COM: self.color_base = COLOR_COMMERCIAL; self.type_str = "COM"
        elif self.priority == PRIORITY_MIL: self.color_base = COLOR_MILITARY; self.type_str = "MIL"
        elif self.priority == PRIORITY_MED: self.color_base = COLOR_MEDICAL; self.type_str = "MED"
        self.id_str = f"{self.type_str}-{uid_num:02d}"  
        self.final_target = None   
        self.stuck_timer = 0      
        self.yield_timer = 0      
        self.temp_avoid_spots = set()   
        self.is_overflying = False    
        self.last_pos = self.pos.copy() 
        self.check_progress_timer = 0   
        self.last_dist_to_target = float('inf') 
        self.patience_timer = 0  

    def set_plan(self, path_points, target_pos):
        if not self.alive: return
        self.path = path_points
        self.final_target = np.array(target_pos)
        self.status = "READY" 
        self.stuck_timer = 0
        self.check_progress_timer = 0
        self.patience_timer = 0
        self.temp_avoid_spots = set()
        self.last_dist_to_target = np.linalg.norm(self.pos - self.final_target) 

    def launch(self):
        if self.status == "READY" and self.alive:
            self.status = "MOVING"

    def kill(self):
        if self.alive:
            self.alive = False
            self.status = "DESTROYED"
            self.vel *= 0
            self.path = []
            print(f"💀 {self.id_str} CRASHED")

    def force_reroute(self, pathfinder, buildings, reason="STUCK"):
        print(f"⚠️ {self.id_str} {reason}. Rerouting...")
        self.status = "REROUTING"
        grid_pos = (int(self.pos[0] // CELL_SIZE), int(self.pos[1] // CELL_SIZE))
        self.temp_avoid_spots.add(grid_pos)
         
        found_safe_spot = False
        for _ in range(10):
            test_move = np.array([random.uniform(-30, 30), random.uniform(-30, 30)])
            test_pos = self.pos + test_move
            safe = True
            test_hitbox = pygame.Rect(test_pos[0]-2, test_pos[1]-2, 4, 4)
            for b in buildings:
                if b.rect.colliderect(test_hitbox):
                    safe = False
                    break
            if safe:
                self.pos = test_pos
                found_safe_spot = True
                break
         
        target_tuple = (self.final_target[0], self.final_target[1])
        new_path = pathfinder.search(self.pos, target_tuple, self.temp_avoid_spots)
        
        if new_path:
            self.path = new_path
            self.stuck_timer = 0
            self.patience_timer = 0
            self.last_dist_to_target = np.linalg.norm(self.pos - self.final_target)
            print(f" {self.id_str} Route Updated!")
        else:
            print(f" {self.id_str} No Route Found.")
            self.status = "IDLE" 
    def update(self, all_drones, buildings, pathfinder):
        if not self.alive: return
         
        core_hitbox = pygame.Rect(self.pos[0]-2, self.pos[1]-2, 4, 4)
        for b in buildings:
            if b.rect.colliderect(core_hitbox):
                self.kill()
                return

        if self.status not in ["MOVING", "REROUTING", "YIELDING"]: return
 
        must_yield = False
        is_peer_deadlock = False
        
        if self.yield_timer > 0:
            self.yield_timer -= 1
            must_yield = True
        
        if self.yield_timer == 0:
            for other in all_drones:
                if other.status == "YIELDING": continue
                if other != self and other.alive and other.status not in ["LANDED", "IDLE"]:
                    dist = np.linalg.norm(self.pos - other.pos)
                    
                    if dist < 80: 
                        # Case A: Priority Yield
                        if other.priority > self.priority:
                            must_yield = True
                            self.yield_timer = 30
                            self.patience_timer = 0
                            break
                        # Case B: Same Priority Tie-Breaker
                        elif other.priority == self.priority and other.id_num < self.id_num:
                             if dist < 50:
                                must_yield = True
                                self.yield_timer = 30
                                if np.linalg.norm(other.vel) < 0.5:
                                    is_peer_deadlock = True
                                else:
                                    is_peer_deadlock = False
                                break
         
        if must_yield:
            if is_peer_deadlock:
                self.patience_timer += 1
                if self.patience_timer > 120:  
                    must_yield = False  
            else:
                self.patience_timer = 0
        else:
            self.patience_timer = 0
 
        if must_yield:
            if self.status != "YIELDING": self.prev_status = self.status
            self.status = "YIELDING"
            self.vel = np.array([0.0, 0.0])
            return
        else:
            if self.status == "YIELDING":
                self.status = "MOVING" if self.prev_status == "IDLE" else self.prev_status
 
        if np.linalg.norm(self.pos - self.last_pos) < 0.5:
            self.stuck_timer += 1
        else:
            self.stuck_timer = 0
            self.last_pos = self.pos.copy()

        self.check_progress_timer += 1
        if self.check_progress_timer > 120:
            if self.final_target is not None:
                curr_dist = np.linalg.norm(self.pos - self.final_target)
                if curr_dist > self.last_dist_to_target - 30:
                    self.force_reroute(pathfinder, buildings, "OSCILLATION")
                    return
                self.last_dist_to_target = curr_dist
            self.check_progress_timer = 0

        if self.stuck_timer > 90 and self.final_target is not None:
            self.force_reroute(pathfinder, buildings, "STUCK")
            return

       
        
        # A. Wall Repulsion
        steer_wall = np.array([0.0, 0.0])
        for b in buildings:
            cx = max(b.rect.left, min(self.pos[0], b.rect.right))
            cy = max(b.rect.top, min(self.pos[1], b.rect.bottom))
            closest = np.array([cx, cy])
            dist = np.linalg.norm(self.pos - closest)
            if dist < WALL_MARGIN:
                push = self.pos - closest
                if np.linalg.norm(push) > 0:
                    push = push / np.linalg.norm(push)
                    strength = (WALL_MARGIN - dist) / WALL_MARGIN
                    steer_wall += push * strength * 4.0

        # B. Path Following
        steer_path = np.array([0.0, 0.0])
        if self.path:
            target = np.array(self.path[0])
            desired = target - self.pos
            dist = np.linalg.norm(desired)
            if dist < 15:
                self.path.pop(0)
                if not self.path:
                    self.status = "LANDED"
                    self.vel *= 0.0
                    return
            if dist > 0: desired = (desired / dist) * self.max_speed
            steer_path = desired - self.vel

        # C. Separation (Overflying Logic)
        steer_sep = np.array([0.0, 0.0])
        self.is_overflying = False
        count = 0
        for other in all_drones:
            if other != self and other.alive:
                safe = False
                is_stopped = np.linalg.norm(other.vel) < 0.1 
                if other.status in ["LANDED", "YIELDING"] or is_stopped:
                    safe = True
                
                if safe and np.linalg.norm(self.pos - other.pos) < 15:
                    self.is_overflying = True
                 
                if not safe:
                    d = np.linalg.norm(self.pos - other.pos)
                    if 0 < d < SEPARATION_DIST:
                        diff = self.pos - other.pos
                        diff = diff / d
                        steer_sep += diff
                        count += 1
        
        if count > 0:
            steer_sep = (steer_sep / count) * self.max_speed
            steer_sep -= self.vel

        # D. Apply Forces
        self.vel += (steer_path * 0.6) + (steer_sep * 1.2) + (steer_wall * 2.5)
        
        speed = np.linalg.norm(self.vel)
        if speed > self.max_speed:
             self.vel = (self.vel / speed) * self.max_speed
        
        self.pos += self.vel

    def draw(self, screen):
        color = self.color_base
        if not self.alive: color = COLOR_DEAD
        elif self.status == "READY": color = COLOR_READY
        elif self.status == "LANDED": color = COLOR_LANDED
        elif self.status == "REROUTING": color = COLOR_REROUTE
        elif self.status == "YIELDING": color = COLOR_YIELD

        if self.alive and len(self.path) > 1:
            lc = color if self.status != "LANDED" else (40, 40, 40)
            pygame.draw.lines(screen, lc, False, [(self.pos[0], self.pos[1])] + self.path, 1)

        if self.selected:
            pygame.draw.rect(screen, SELECTION_COLOR, (self.pos[0]-10, self.pos[1]-10, 20, 20), 1)

        pygame.draw.circle(screen, color, (int(self.pos[0]), int(self.pos[1])), 6)
        
        font = pygame.font.SysFont("Arial", 10, bold=True)
        if self.is_overflying:
            lbl = font.render("OVERFLY", True, COLOR_OVERFLY)
            screen.blit(lbl, (self.pos[0]-20, self.pos[1]-20))
        elif self.status == "YIELDING":
            lbl = font.render("WAIT", True, COLOR_YIELD)
            screen.blit(lbl, (self.pos[0]-15, self.pos[1]-20))
        elif self.status == "REROUTING":
            lbl = font.render("REROUTE", True, COLOR_REROUTE)
            screen.blit(lbl, (self.pos[0]-25, self.pos[1]-20))
        elif self.status == "READY":
            lbl = font.render("READY", True, COLOR_READY)
            screen.blit(lbl, (self.pos[0]-15, self.pos[1]-20))