import pygame
import numpy as np
import heapq
import random

# --- CONFIGURATION ---
WIDTH, HEIGHT = 1600, 900
CELL_SIZE = 20

# COLORS
BG_COLOR = (0, 0, 0)
ROAD_COLOR = (25, 25, 25)
BUILDING_FILL = (0, 30, 0)
BUILDING_OUTLINE = (0, 255, 0)

# STATUS COLORS
COLOR_IDLE = (200, 200, 200)
COLOR_READY = (255, 255, 0)
COLOR_MOVING = (0, 255, 255)
COLOR_LANDED = (0, 100, 50)
COLOR_YIELD = (255, 165, 0)
COLOR_DEAD = (80, 80, 80)
COLOR_REROUTE = (255, 0, 255) # Magenta for "Thinking/Rerouting"
COLOR_OVERFLY = (255, 255, 255)

SELECTION_COLOR = (255, 0, 0)

# --- A* PATHFINDING ---
class Pathfinder:
    def __init__(self, width, height, cell_size):
        self.cell_size = cell_size
        self.cols = width // cell_size
        self.rows = height // cell_size
        self.obstacles = set()

    def add_obstacle(self, rect):
        padding = 1 
        start_c = max(0, (rect.left // self.cell_size) - padding)
        end_c = min(self.cols, (rect.right // self.cell_size) + 1 + padding)
        start_r = max(0, (rect.top // self.cell_size) - padding)
        end_r = min(self.rows, (rect.bottom // self.cell_size) + 1 + padding)
        for c in range(start_c, end_c):
            for r in range(start_r, end_r):
                self.obstacles.add((c, r))

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def find_nearest_road(self, node):
        if node not in self.obstacles: return node
        for r in range(1, 10):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    neighbor = (node[0] + dx, node[1] + dy)
                    if (0 <= neighbor[0] < self.cols and 0 <= neighbor[1] < self.rows and neighbor not in self.obstacles):
                        return neighbor
        return None

    def search(self, start_pos, end_pos):
        start = (int(start_pos[0] // self.cell_size), int(start_pos[1] // self.cell_size))
        raw_end = (int(end_pos[0] // self.cell_size), int(end_pos[1] // self.cell_size))
        end = self.find_nearest_road(raw_end)
        if not end: return []
        if start in self.obstacles: start = self.find_nearest_road(start)
        if not start: return []

        queue = []
        heapq.heappush(queue, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while queue:
            current = heapq.heappop(queue)[1]
            if current == end: break
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                next_node = (current[0] + dx, current[1] + dy)
                if (0 <= next_node[0] < self.cols and 0 <= next_node[1] < self.rows and next_node not in self.obstacles):
                    new_cost = cost_so_far[current] + 1
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + self.heuristic(end, next_node)
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

# --- VISUALS ---
class Building:
    def __init__(self, x, y, w, h):
        self.rect = pygame.Rect(x, y, w, h)
    def draw(self, screen):
        pygame.draw.rect(screen, BUILDING_FILL, self.rect)
        pygame.draw.rect(screen, BUILDING_OUTLINE, self.rect, 1)

# --- DRONE WITH REROUTING CAPABILITY ---
class Drone:
    def __init__(self, x, y, uid_num):
        self.id_num = uid_num
        self.id_str = f"D{uid_num:02d}"
        self.pos = np.array([float(x), float(y)])
        self.vel = np.array([0.0, 0.0])
        self.max_speed = 4.0 
        self.path = []
        self.selected = False
        self.status = "IDLE" 
        self.alive = True
        self.is_overflying = False
        
        # NAVIGATION MEMORY
        self.final_target = None # Remember where we are going
        self.stuck_timer = 0
        self.last_pos = self.pos.copy()

    def set_plan(self, path_points, target_pos):
        if not self.alive: return
        self.path = path_points
        self.final_target = target_pos # Save destination coordinates
        self.status = "READY"
        self.stuck_timer = 0

    def launch(self):
        if self.status == "READY" and self.alive:
            self.status = "MOVING"

    def kill(self):
        if self.alive:
            self.alive = False
            self.status = "DESTROYED"
            self.vel = np.array([0.0, 0.0]) 
            self.path = []
            print(f"💀 UNIT {self.id_str} DESTROYED")

    # UPDATE NOW ACCEPTS PATHFINDER
    def update(self, all_drones, buildings, pathfinder):
        if not self.alive: return 

        # COLLISION CHECK
        body_hitbox = pygame.Rect(self.pos[0]-5, self.pos[1]-5, 10, 10)
        for b in buildings:
            if b.rect.colliderect(body_hitbox):
                self.kill()
                return

        if self.status != "MOVING": return

        # --- INTELLIGENT REROUTING ---
        # 1. Detect if we are stuck (Moved < 2 pixels in 1 second)
        if np.linalg.norm(self.pos - self.last_pos) < 2.0:
            self.stuck_timer += 1
        else:
            self.stuck_timer = 0
            self.last_pos = self.pos.copy()

        # 2. If stuck for > 60 frames (1 second), REROUTE
        if self.stuck_timer > 60 and self.final_target:
            self.status = "REROUTING" # Visual Feedback
            print(f"⚠️ {self.id_str} STUCK. Calculating new route...")
            
            # Wiggle back slightly to get out of the wall collision zone
            self.pos -= self.vel * 5.0 
            
            # Request new path from CURRENT position to FINAL target
            new_path = pathfinder.search(self.pos, self.final_target)
            
            if new_path:
                self.path = new_path
                self.stuck_timer = 0
                self.status = "MOVING" # Resume immediately
                print(f"🔄 {self.id_str} New Route Acquired!")
            else:
                print(f"❌ {self.id_str} Reroute Failed (No Path).")
            return

        # --- MOVEMENT (Same as before) ---
        self.is_overflying = False
        for other in all_drones:
            if other != self and other.status == "LANDED":
                if np.linalg.norm(self.pos - other.pos) < 15:
                    self.is_overflying = True

        # Wall Avoidance
        steer_avoid_wall = np.array([0.0, 0.0])
        wall_margin = 25.0 
        for b in buildings:
            closest_x = max(b.rect.left, min(self.pos[0], b.rect.right))
            closest_y = max(b.rect.top, min(self.pos[1], b.rect.bottom))
            closest_pt = np.array([closest_x, closest_y])
            dist = np.linalg.norm(self.pos - closest_pt)
            if dist < wall_margin:
                push = (self.pos - closest_pt) 
                if np.linalg.norm(push) > 0:
                    push = push / np.linalg.norm(push)
                    strength = (wall_margin - dist) ** 2 
                    steer_avoid_wall += push * strength * 0.2

        # Path Following
        steer_path = np.array([0.0, 0.0])
        if self.path:
            target = np.array(self.path[0])
            desired = target - self.pos
            dist = np.linalg.norm(desired)
            
            if dist < 20:
                self.path.pop(0)
                if not self.path:
                    self.status = "LANDED"
                    self.vel *= 0.0
                    print(f"✅ {self.id_str}: LANDED AT TARGET")
                    return

            if dist > 0: desired = (desired / dist) * self.max_speed
            steer_path = desired - self.vel
            if np.linalg.norm(steer_path) > 0.2:
                steer_path = (steer_path / np.linalg.norm(steer_path)) * 0.2

        # Separation
        steer_sep = np.array([0.0, 0.0])
        count = 0
        for other in all_drones:
            if other != self and other.alive and other.status != "LANDED":
                d = np.linalg.norm(self.pos - other.pos)
                if 0 < d < 30:
                    diff = self.pos - other.pos
                    diff = diff / d 
                    steer_sep += diff
                    count += 1
        if count > 0:
            steer_sep = (steer_sep / count) * self.max_speed
            steer_sep -= self.vel

        self.vel += steer_path + steer_sep + steer_avoid_wall
        speed = np.linalg.norm(self.vel)
        if speed > self.max_speed:
             self.vel = (self.vel / speed) * self.max_speed
        self.pos += self.vel

    def draw(self, screen):
        color = COLOR_IDLE
        if not self.alive: color = COLOR_DEAD
        elif self.status == "READY": color = COLOR_READY
        elif self.status == "MOVING": color = COLOR_MOVING
        elif self.status == "LANDED": color = COLOR_LANDED
        elif self.status == "YIELDING": color = COLOR_YIELD
        elif self.status == "REROUTING": color = COLOR_REROUTE # Magenta

        if self.alive and len(self.path) > 1:
            line_color = color
            if self.status == "LANDED": line_color = (50, 50, 50) 
            pygame.draw.lines(screen, line_color, False, [(self.pos[0], self.pos[1])] + self.path, 1)

        if self.selected:
            pygame.draw.rect(screen, SELECTION_COLOR, (self.pos[0]-12, self.pos[1]-12, 24, 24), 1)

        pygame.draw.circle(screen, color, (int(self.pos[0]), int(self.pos[1])), 6)
        
        # LABELS
        font = pygame.font.SysFont("Consolas", 10, bold=True)
        
        if not self.alive:
            lbl = font.render("LOST", True, COLOR_DEAD)
            screen.blit(lbl, (self.pos[0]-15, self.pos[1]-20))
        elif self.is_overflying:
            lbl = font.render("OVERFLYING", True, COLOR_OVERFLY)
            screen.blit(lbl, (self.pos[0]-25, self.pos[1]-25))
        elif self.status == "LANDED":
            lbl = font.render("REST", True, COLOR_LANDED)
            screen.blit(lbl, (self.pos[0]-12, self.pos[1]-20))
        elif self.status == "REROUTING":
            lbl = font.render("REROUTING", True, COLOR_REROUTE)
            screen.blit(lbl, (self.pos[0]-25, self.pos[1]-20))

# --- MAIN ENGINE ---
def main():
    pygame.init()
    info = pygame.display.Info()
    SCREEN_W, SCREEN_H = info.current_w - 50, info.current_h - 50
    global WIDTH, HEIGHT
    WIDTH, HEIGHT = SCREEN_W, SCREEN_H
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("SWARM_X | Auto-Reroute Capability")
    clock = pygame.time.Clock()

    pathfinder = Pathfinder(WIDTH, HEIGHT, CELL_SIZE)
    buildings = []
    drones = []

    print("Generating City...")
    BLOCK_SIZE = 120 
    STREET_WIDTH = 70 
    
    for x in range(50, WIDTH - 50, BLOCK_SIZE + STREET_WIDTH):
        for y in range(50, HEIGHT - 50, BLOCK_SIZE + STREET_WIDTH):
            if random.random() > 0.15:
                b_w = BLOCK_SIZE - random.randint(0, 20)
                b_h = BLOCK_SIZE - random.randint(0, 20)
                offset_x = (BLOCK_SIZE - b_w) // 2
                offset_y = (BLOCK_SIZE - b_h) // 2
                rect = pygame.Rect(x + offset_x, y + offset_y, b_w, b_h)
                buildings.append(Building(rect.x, rect.y, rect.width, rect.height))
                pathfinder.add_obstacle(rect)

    selected_drone = None

    running = True
    while running:
        screen.fill(BG_COLOR)
        for x in range(0, WIDTH, 100): pygame.draw.line(screen, ROAD_COLOR, (x, 0), (x, HEIGHT), 1)
        for y in range(0, HEIGHT, 100): pygame.draw.line(screen, ROAD_COLOR, (0, y), (WIDTH, y), 1)

        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                
                if event.button == 1:
                    clicked_drone = False
                    for d in drones:
                        if np.linalg.norm(d.pos - np.array([mx, my])) < 20:
                            if selected_drone: selected_drone.selected = False
                            selected_drone = d
                            d.selected = True
                            clicked_drone = True
                            break
                    if not clicked_drone and selected_drone: pass 

                elif event.button == 3 and selected_drone:
                    # Pass TUPLE (mx, my) as target to set_plan
                    path = pathfinder.search(selected_drone.pos, (mx, my))
                    if path: selected_drone.set_plan(path, (mx, my))
                
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    print("🚀 ENGAGING...")
                    for d in drones: d.launch()

                if event.key == pygame.K_SPACE:
                    mx, my = pygame.mouse.get_pos()
                    drones.append(Drone(mx, my, len(drones)+1)) 

        for b in buildings: b.draw(screen)
        
        for d in drones:
            # PASS PATHFINDER TO UPDATE
            d.update(drones, buildings, pathfinder)

        # Draw Landed
        for d in drones:
            if d.status == "LANDED" or not d.alive: d.draw(screen)
        # Draw Flying
        for d in drones:
            if d.status != "LANDED" and d.alive: d.draw(screen)

        # UI
        pygame.draw.rect(screen, (10, 20, 10), (0, HEIGHT-35, WIDTH, 35))
        font = pygame.font.SysFont("Consolas", 16, bold=True)
        txt = font.render(f"UNITS: {len(drones)} | SPACE: SPAWN | ENTER: ENGAGE | INTELLIGENT REROUTE: ON", True, (0, 255, 0))
        screen.blit(txt, (20, HEIGHT - 28))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()