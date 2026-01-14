import pygame    
import numpy as np  
import heapq    
import random   
import math      
WIDTH, HEIGHT = 1600, 900
CELL_SIZE = 20   
BG_COLOR = (10, 10, 15)     
ROAD_COLOR = (25, 25, 30)   
BUILDING_FILL = (15, 25, 20)  
BUILDING_OUTLINE = (0, 180, 80)   
COLOR_COMMERCIAL = (0, 255, 0)  
COLOR_MILITARY = (0, 0, 255)   
COLOR_MEDICAL = (255, 0, 0)   
COLOR_READY = (255, 255, 0)       
COLOR_LANDED = (40, 60, 40)      
COLOR_YIELD = (255, 140, 0)      
COLOR_DEAD = (50, 50, 50)       
COLOR_REROUTE = (255, 0, 255)     
COLOR_OVERFLY = (255, 255, 255)   
SELECTION_COLOR = (255, 255, 255)   

#----------------------------------------------------------
class Pathfinder:
    def __init__(self, width, height, cell_size):
        self.cell_size = cell_size  
        self.cols = width // cell_size          
        self.rows = height // cell_size 
        self.obstacles = set()     #here we are using a set to store obstacles for faster lookup
    def add_obstacle(self, rect):
        padding = 0
        start_c = max(0, (rect.left // self.cell_size) - padding)
        end_c = min(self.cols, (rect.right // self.cell_size) + 1 + padding)
        start_r = max(0, (rect.top // self.cell_size) - padding)
        end_r = min(self.rows, (rect.bottom // self.cell_size) + 1 + padding)
        for c in range(start_c, end_c):
            for r in range(start_r, end_r):
                self.obstacles.add((c, r)) 
    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)      #shortest distance btw the current node and the  target node

    def find_nearest_road(self,node):       #here were are checking by whether node is in obstacle or not if it is not in obstacle we return that node otherwise we  check  for the  nearest safe coordinate
        if node not in self.obstacles: return node 
        for r in range(1, 15):  #we assumed the radius of 15 cells to search for nearest road
            for dx in range(-r, r+1):  #we are checking in square pattern of size max 30x30
                for dy in range(-r, r+1): 
                    neighbor = (node[0] + dx, node[1] + dy)
                    if (0 <= neighbor[0] < self.cols and 0 <= neighbor[1] < self.rows and neighbor not in self.obstacles):
                        return neighbor
        return None 
    #-----------------------------------------------------------------------------|
     # A* SEARCH ALGORITHM--------------------------------------------------------|
    def search(self, start_pos, end_pos, temp_obstacles=None):
        start = (int(start_pos[0] // self.cell_size), int(start_pos[1] // self.cell_size))
        raw_end = (int(end_pos[0] // self.cell_size), int(end_pos[1] // self.cell_size))
         
        end = self.find_nearest_road(raw_end)      
        if not end: return [] 
        if start in self.obstacles: start = self.find_nearest_road(start)
        if not start: return []
    
        queue = []
        heapq.heappush(queue, (0, start))       #priority queue initialized with start node ,it stores tuples of (priority,cordinate)
        came_from = {start: None}   
        cost_so_far = {start: 0}    #dictionary to store cost from start to each node
        current_obstacles = self.obstacles 
        if temp_obstacles:
            current_obstacles = self.obstacles.union(temp_obstacles)      #temorary obstacles are the spots to avoid for rerouting , these are the spots where the drone got stuck previously
        while queue:
            current = heapq.heappop(queue)[1] #get the node with lowest priority
            if current == end: break 
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                next_node = (current[0] + dx, current[1] + dy)
                if (0 <= next_node[0] < self.cols and 0 <= next_node[1] < self.rows and next_node not in current_obstacles):
                    move_cost = 1.42 if dx != 0 and dy != 0 else 1.0
                    new_cost = cost_so_far[current] + move_cost
                    
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost #update cost to reach next_node
                        priority = new_cost + self.heuristic(end, next_node) #calculate priority 
                        heapq.heappush(queue, (priority, next_node)) 
                        came_from[next_node] = current

        if end not in came_from: return [] 
        path = []
        curr = end
        while curr != start:
            pixel_pos = (curr[0] * self.cell_size + self.cell_size//2, curr[1] * self.cell_size + self.cell_size//2)
            path.append(pixel_pos)
            curr = came_from[curr]
            if curr is None: break
        path.reverse() 
        return path
##################################################################################################################################
# --- VISUALS ---
class Building:
    def __init__(self, x, y, w, h):     
        self.rect = pygame.Rect(x, y, w, h) 
    def draw(self, screen):
        pygame.draw.rect(screen, BUILDING_FILL, self.rect)   
        pygame.draw.rect(screen, BUILDING_OUTLINE, self.rect, 1) 

# --- DRONE AGENT ---
class Drone:
    def __init__(self, x, y, uid_num, fixed_priority):
        self.id_num = uid_num   
        self.pos = np.array([float(x), float(y)]) 
        self.vel = np.array([0.0, 0.0]) 
        self.max_speed = 3.5    
        self.path = []          
        self.selected = False   
        self.status = "IDLE"    
        self.alive = True       
        self.prev_status = "IDLE" 
        
        # Priority System
        self.priority = fixed_priority
        if self.priority == 1: self.color_base = COLOR_COMMERCIAL; self.type_str = "COM"
        elif self.priority == 2: self.color_base = COLOR_MILITARY; self.type_str = "MIL"
        elif self.priority == 3: self.color_base = COLOR_MEDICAL; self.type_str = "MED"
        self.id_str = f"{self.type_str}-{uid_num:02d}"  
        # Navigation Memory
        self.final_target = None  
        self.stuck_timer = 0      
        self.yield_timer = 0      
        self.temp_avoid_spots = set() 
        self.is_overflying = False    
        self.last_pos = self.pos.copy() 
        self.check_progress_timer = 0   
        self.last_dist_to_target = float('inf') 
        
        # NEW: Patience Timer for Deadlocks
        self.patience_timer = 0  
    def set_plan(self, path_points, target_pos):
        if not self.alive: return
        self.path = path_points
        self.final_target = np.array(target_pos)
        self.status = "READY" 
        self.stuck_timer = 0
        self.check_progress_timer = 0
        self.patience_timer = 0 # Reset patience
        self.temp_avoid_spots = set()
        self.last_dist_to_target = np.linalg.norm(self.pos - self.final_target)

    def launch(self):
        if self.status == "READY" and self.alive: #self.alive checks if drone is not destroyed if it is true then only it can launch
            self.status = "MOVING"

    def kill(self):
        if self.alive:
            self.alive = False
            self.status = "DESTROYED"
            self.vel *= 0 
            self.path = [] 
            print(f"💀 {self.id_str} CRASHED")
#---------------------------------------------------------------------------------------------tested till here
    def force_reroute(self, pathfinder, buildings, reason="STUCK"):
        print(f"⚠️ {self.id_str} {reason}. Rerouting...")
        self.status = "REROUTING" 
        grid_pos = (int(self.pos[0] // CELL_SIZE), int(self.pos[1] // CELL_SIZE)) #current  possition of drones in  the  form  of coordinates
        self.temp_avoid_spots.add(grid_pos)  
        found_safe_spot = False
        for _ in range(10): 
            test_move = np.array([random.uniform(-30, 30), random.uniform(-30, 30)]) #random  move vector within  the  range of -30 to 30 pixels
            test_pos = self.pos + test_move 
            safe = True 
            test_hitbox = pygame.Rect(test_pos[0]-2, test_pos[1]-2, 4, 4)#what its doing is creating a small hitbox around the test position to check for collisions with buildings
            for b in buildings:
                if b.rect.colliderect(test_hitbox): #here we are checking for collision between the test hitbox and building rectangle if they collide then we mark that position as unsafe
                    safe = False
                    break 
            if safe:
                self.pos = test_pos  
                found_safe_spot = True 
                break
        
        if not found_safe_spot:
            print(f"⚠️ {self.id_str} Cannot jiggle safely, recalculating in place.")

        target_tuple = (self.final_target[0], self.final_target[1])
        new_path = pathfinder.search(self.pos, target_tuple, self.temp_avoid_spots)
        
        if new_path:
            self.path = new_path
            self.stuck_timer = 0
            self.check_progress_timer = 0
            self.patience_timer = 0
            self.last_dist_to_target = np.linalg.norm(self.pos - self.final_target)
            print(f"🔄 {self.id_str} Route Updated!")
        else:
            print(f"❌ {self.id_str} No Route Found.")
            self.status = "IDLE"  
    def update(self, all_drones, buildings, pathfinder):  #it updates the drone's state each frame ,basically it is the main  function  due to  which  we sees the drone moving around the speed of movement of drone depends upon  how fast this function  is  calling
        if not self.alive: return  # If drone is destroyed, do nothing
        # 1. HARD COLLISION (Core Hitbox)
        core_hitbox = pygame.Rect(self.pos[0]-2, self.pos[1]-2, 4, 4) #core hitbox is a small rectangle around the drone's position to check for collisions with buildings
        for b in buildings:
            if b.rect.colliderect(core_hitbox): 
                self.kill()
                return 
        if self.status not in ["MOVING", "REROUTING", "YIELDING"]: return 
        # --- YIELDING & DEADLOCK LOGIC ---
        must_yield = False # Initial assumption that we do not need to yield ,here yield means to stop and wait for other drones to pass
        is_peer_deadlock = False # Flag for deadlock with peer drone that is also stuck ,here deadlock means both drones are waiting for each other and neither can proceed
        if self.yield_timer > 0:
            self.yield_timer -= 1
            must_yield = True 
        
        if self.yield_timer == 0:
            for other in all_drones:
                # FIX 1: Do not yield to a drone that is already waiting!
                if other.status == "YIELDING": continue 
                
                if other != self and other.alive and other.status not in ["LANDED", "IDLE"]:
                    dist = np.linalg.norm(self.pos - other.pos) #calculate distance between self and other drone if the  distance is less than 100 pixels then we check for priority
                    
                    if dist < 100: 
                        # Case A: Strict Priority (Always yield to higher rank)
                        if other.priority > self.priority:
                            must_yield = True
                            self.yield_timer = 30 
                            self.patience_timer = 0 
                            break
                        
                        # Case B: Same Priority (Tie-Breaker)
                        elif other.priority == self.priority and other.id_num < self.id_num:
                             if dist < 50: 
                                must_yield = True
                                self.yield_timer = 30
                                
                                # Check for Peer Deadlock (Are they stuck too?)
                                if np.linalg.norm(other.vel) < 0.5:
                                    is_peer_deadlock = True
                                else:
                                    is_peer_deadlock = False
                                break
        
        # --- PATIENCE CHECKER ---
        if must_yield:
            if is_peer_deadlock:
                self.patience_timer += 1
                if self.patience_timer > 120: # 2 Seconds
                    must_yield = False # Break the rules and go
            else:
                self.patience_timer = 0 
        else:
            self.patience_timer = 0 
        # --- APPLY STATE ---
        if must_yield:
            if self.status != "YIELDING": 
                self.prev_status = self.status # Remember previous state before yielding 
            self.status = "YIELDING"  
            self.vel = np.array([0.0, 0.0]) 
            return 
        else:
            if self.status == "YIELDING": 
                self.status = "MOVING" if self.prev_status == "IDLE" else self.prev_status

        # --- STUCK CHECK ---
        if np.linalg.norm(self.pos - self.last_pos) < 0.5:
            self.stuck_timer += 1 
        else:
            self.stuck_timer = 0
            self.last_pos = self.pos.copy() 

        self.check_progress_timer += 1
        if self.check_progress_timer > 120: 
            if self.final_target is not None:
                curr_dist = np.linalg.norm(self.pos - self.final_target)
                if curr_dist > self.last_dist_to_target - 30:# it checks if the drone has made significant progress towards its final target over the last interval. If not, it considers itself stuck in an oscillation.
                    self.force_reroute(pathfinder, buildings, "OSCILLATION")
                    return     
                self.last_dist_to_target = curr_dist  
            self.check_progress_timer = 0 
        if self.stuck_timer > 90 and self.final_target is not None:
            self.force_reroute(pathfinder, buildings, "STUCK")
            return 
        # --- PHYSICS ENGINE ---
        steer_wall = np.array([0.0, 0.0])
        wall_margin = 20.0 
        for b in buildings:
            cx = max(b.rect.left, min(self.pos[0], b.rect.right))
            cy = max(b.rect.top, min(self.pos[1], b.rect.bottom))
            closest = np.array([cx, cy])    
            dist = np.linalg.norm(self.pos - closest)# if the drone is too close to the building wall ,it calculates a repulsion force to push it away from the wall
            if dist < wall_margin: # it checcks if the distance is less than the wall margin ,meaning the drone is too close to the wall ,meaning it needs to be pushed away or we can say that the position of drone is inside the wall margin
                push = self.pos - closest# calculate push vector away from wall ,direction from wall to drone
                if np.linalg.norm(push) > 0: #avoid division by zero
                    push = push / np.linalg.norm(push) #normalize push vector we want only the direction not the magnitude ,its basically the unit vector in the direction of push
                    strength = (wall_margin - dist) / wall_margin #calculate strength of push based on how close we are to the wall ,the closer we are the stronger the push
                    steer_wall += push * strength * 4.0              
        steer_path = np.array([0.0, 0.0]) 
        if self.path: # if there is a path to follow
            target = np.array(self.path[0]) #get next waypoint
            desired = target - self.pos #calculate desired velocity towards waypoint
            dist = np.linalg.norm(desired) #then we calculate the distance to that waypoint 
            if dist < 15: 
                self.path.pop(0) 
                if not self.path:
                    self.status = "LANDED"
                    self.vel *= 0.0
                    return
            if dist > 0: desired = (desired / dist) * self.max_speed #it normalizes the desired vector and scales it to max speed
            steer_path = desired - self.vel #it calculates the steering force needed to adjust current velocity towards desired velocity

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
                # Only repel if they are MOVING and creating a collision risk
                if not safe:
                    d = np.linalg.norm(self.pos - other.pos)
                    if 0 < d < 40: 
                        diff = self.pos - other.pos
                        diff = diff / d
                        steer_sep += diff
                        count += 1 
        if count > 0:
            steer_sep = (steer_sep / count) * self.max_speed
            steer_sep -= self.vel 
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

# --- MAIN SIMULATION ---
def main():
    pygame.init()
    info = pygame.display.Info()
    SCREEN_W, SCREEN_H = info.current_w - 50, info.current_h - 50
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = SCREEN_W, SCREEN_H
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("SWARM_X | Deadlock Breaker Edition")
    clock = pygame.time.Clock()

    pathfinder = Pathfinder(WIDTH, HEIGHT, CELL_SIZE)
    buildings = []
    drones = []

    print("Generating City...")
    MAIN_ROAD_WIDTH = 80
    BLOCK_SIZE = 250
    
    for bx in range(50, WIDTH - 50, BLOCK_SIZE + MAIN_ROAD_WIDTH):
        for by in range(50, HEIGHT - 50, BLOCK_SIZE + MAIN_ROAD_WIDTH):
            current_x = bx
            while current_x < bx + BLOCK_SIZE:
                b_width = random.randint(40, 100)
                if current_x + b_width > bx + BLOCK_SIZE: b_width = (bx + BLOCK_SIZE) - current_x
                current_y = by
                while current_y < by + BLOCK_SIZE:
                    b_height = random.randint(40, 100)
                    if current_y + b_height > by + BLOCK_SIZE: b_height = (by + BLOCK_SIZE) - current_y
                    if random.random() > 0.1:
                        gap = random.randint(2, 10) 
                        if b_width > gap*2 and b_height > gap*2:
                            rect = pygame.Rect(current_x + gap, current_y + gap, b_width - gap*2, b_height - gap*2)
                            buildings.append(Building(rect.x, rect.y, rect.width, rect.height))
                            pathfinder.add_obstacle(rect)
                    current_y += b_height
                current_x += b_width

    selected_drone = None
    spawn_mode = 1 # 1=Com, 2=Mil, 3=Med

    running = True
    while running:
        screen.fill(BG_COLOR) 
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                if event.button == 1:
                    clicked = False
                    for d in drones:
                        if np.linalg.norm(d.pos - np.array([mx, my])) < 20:
                            if selected_drone: selected_drone.selected = False
                            selected_drone = d
                            d.selected = True
                            clicked = True
                            break
                    if not clicked and selected_drone: pass 
                elif event.button == 3 and selected_drone:
                    path = pathfinder.search(selected_drone.pos, (mx, my))
                    if path: selected_drone.set_plan(path, (mx, my))
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    for d in drones: d.launch()
                
                # Mode Selection
                if event.key == pygame.K_1: spawn_mode = 1
                if event.key == pygame.K_2: spawn_mode = 2
                if event.key == pygame.K_3: spawn_mode = 3

                # Spawn
                if event.key == pygame.K_SPACE:
                    mx, my = pygame.mouse.get_pos()
                    safe = True
                    for b in buildings:
                        if b.rect.collidepoint(mx, my): safe = False
                    if safe: drones.append(Drone(mx, my, len(drones)+1, spawn_mode)) 

        for b in buildings: b.draw(screen)
        for d in drones: d.update(drones, buildings, pathfinder)
        
        # Draw Order: Landed -> Flying
        for d in drones:
            if d.status == "LANDED" or not d.alive: d.draw(screen)
        for d in drones:
            if d.status != "LANDED" and d.alive: d.draw(screen)

        # UI Bar
        pygame.draw.rect(screen, (10, 20, 10), (0, HEIGHT-35, WIDTH, 35))
        font = pygame.font.SysFont("Arial", 16, bold=True)
        
        mode_text, mode_color = "COMMERCIAL", COLOR_COMMERCIAL
        if spawn_mode == 2: mode_text, mode_color = "MILITARY", COLOR_MILITARY
        elif spawn_mode == 3: mode_text, mode_color = "MEDICAL", COLOR_MEDICAL

        txt_static = font.render(f"UNITS: {len(drones)} | KEYS [1,2,3] SELECT TYPE: ", True, (200, 200, 200))
        txt_mode = font.render(f"[{mode_text}]", True, mode_color)
        screen.blit(txt_static, (20, HEIGHT - 28))
        screen.blit(txt_mode, (320, HEIGHT - 28))

        pygame.display.flip() 
        clock.tick(60) 

    pygame.quit()

if __name__ == "__main__":
    main()

