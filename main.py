import pygame
import numpy as np
import math

# --- CONFIGURATION (The "Dark Mode" Aesthetic) ---
WIDTH, HEIGHT = 1200, 900
BG_COLOR = (10, 15, 20)      # Deep Navy/Black
GRID_COLOR = (30, 40, 50)    # Faint Grid Lines
RADAR_GREEN = (0, 255, 128)  # Sci-Fi Green
DRONE_WHITE = (220, 220, 255)
TARGET_RED = (255, 50, 50)

# --- THE PHYSICS ENGINE ---
class Drone:
    def __init__(self, x, y, id_num):
        self.id = id_num
        self.pos = np.array([float(x), float(y)])
        self.vel = np.array([0.0, 0.0])
        self.acc = np.array([0.0, 0.0])
        
        # Physics Constants (Tweak these for "feel")
        self.max_speed = 5.0
        self.max_force = 0.15  # Agility (Higher = sharper turns)
        self.target = None

    def set_target(self, x, y):
        self.target = np.array([float(x), float(y)])

    def update(self):
        if self.target is not None:
            # 1. Vector to Target
            desired = self.target - self.pos
            dist = np.linalg.norm(desired)

            # 2. Arrival Logic (Slow down closer to target)
            if dist < 100:
                speed = (dist / 100) * self.max_speed
            else:
                speed = self.max_speed
            
            # 3. Steering Force = Desired - Current Velocity
            if dist > 0:
                desired = (desired / dist) * speed
                
            steer = desired - self.vel
            
            # Limit the force (Inertia)
            if np.linalg.norm(steer) > self.max_force:
                steer = (steer / np.linalg.norm(steer)) * self.max_force

            # Apply Physics
            self.acc = steer
            self.vel += self.acc
            self.pos += self.vel

    def draw(self, screen):
        # Draw Body
        pygame.draw.circle(screen, DRONE_WHITE, (int(self.pos[0]), int(self.pos[1])), 6)
        
        # Draw Velocity Vector (Where it's going)
        end_pos = self.pos + (self.vel * 15)
        pygame.draw.line(screen, RADAR_GREEN, self.pos, end_pos, 2)
        
        # Draw ID Label
        font = pygame.font.SysFont("Consolas", 12)
        label = font.render(f"DRONE-{self.id}", True, RADAR_GREEN)
        screen.blit(label, (self.pos[0] + 10, self.pos[1] - 10))

# --- MAIN SYSTEM LOOP ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("SWARM_X | Tactical Airspace Manager")
    clock = pygame.time.Clock()

    # Spawn our first agent
    drone = Drone(WIDTH//2, HEIGHT//2, "01")
    target = None

    running = True
    while running:
        screen.fill(BG_COLOR)
        
        # Draw Grid (The "Map")
        for x in range(0, WIDTH, 50):
            pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, HEIGHT))
        for y in range(0, HEIGHT, 50):
            pygame.draw.line(screen, GRID_COLOR, (0, y), (WIDTH, y))

        # Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # Mouse Click = New Target
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                target = (mx, my)
                drone.set_target(mx, my)

        # Update & Render
        drone.update()
        
        if target:
            pygame.draw.circle(screen, TARGET_RED, target, 5, 1) # Draw Target Marker
            pygame.draw.line(screen, (50, 50, 50), drone.pos, target, 1) # Draw Path Line

        drone.draw(screen)

        # Refresh
        pygame.display.flip()
        clock.tick(60) # 60 FPS Lock

    pygame.quit()

if __name__ == "__main__":
    main()