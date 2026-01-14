import heapq
import math
from settings import *

class Pathfinder:
    def __init__(self, width, height, cell_size):
        self.cell_size = cell_size
        self.cols = width // cell_size
        self.rows = height // cell_size
        self.obstacles = set()  

    def add_obstacle(self, rect): 
        start_c = max(0, rect.left // self.cell_size)
        end_c = (rect.right // self.cell_size) + 1
        start_r = max(0, rect.top // self.cell_size)
        end_r = (rect.bottom // self.cell_size) + 1 
        for c in range(start_c, end_c):
            for r in range(start_r, end_r):
                self.obstacles.add((c, r))

    def heuristic(self, a, b): 
        return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def find_nearest_road(self, node): 
        if node not in self.obstacles: return node
        for r in range(1, 15):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    neighbor = (node[0] + dx, node[1] + dy)
                    if 0 <= neighbor[0] < self.cols and 0 <= neighbor[1] < self.rows:
                        if neighbor not in self.obstacles:
                            return neighbor
        return None

    def search(self, start_pos, end_pos, temp_obstacles=None): 
        start = (int(start_pos[0] // self.cell_size), int(start_pos[1] // self.cell_size))
        raw_end = (int(end_pos[0] // self.cell_size), int(end_pos[1] // self.cell_size))
        end = self.find_nearest_road(raw_end)
        if not end: return [] 
        
        queue = []
        heapq.heappush(queue, (self.heuristic(start, end), start))
        came_from = {start: None} 
        cost_so_far = {start: 0}
         
        current_obstacles = self.obstacles
        if temp_obstacles:
            current_obstacles = self.obstacles.union(temp_obstacles)
        
        while queue:
            current = heapq.heappop(queue)[1]
            if current == end: break
            
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                next_node = (current[0] + dx, current[1] + dy)
                 
                if 0 <= next_node[0] < self.cols and 0 <= next_node[1] < self.rows:
                    
                    if next_node not in current_obstacles: 
                        move_cost = 1.42 if dx != 0 and dy != 0 else 1.0
                        new_cost = cost_so_far[current] + move_cost 
                         
                        if next_node not in cost_so_far or new_cost < cost_so_far[next_node]: 
                            cost_so_far[next_node] = new_cost 
                            total_cost = new_cost + self.heuristic(end, next_node)
                            heapq.heappush(queue, (total_cost, next_node))
                            came_from[next_node] = current

        if end not in came_from: return []
        
        path = []
        curr = end
        while curr != start: 
            pixel_pos = (curr[0] * self.cell_size + self.cell_size//2, 
                         curr[1] * self.cell_size + self.cell_size//2) 
            path.append(pixel_pos)
            curr = came_from[curr]
            if curr is None: break 
        path.reverse()
        return path