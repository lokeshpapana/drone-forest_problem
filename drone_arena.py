"""
Forest Arena – Graph-based road network, dense trees, dynamic obstacles, A* pathfinding
Controls:
  WASD / Arrow Keys   : Move drone
  Mouse drag          : Pan camera
  Mouse wheel or +/-  : Zoom
  SPACE               : Toggle follow mode
  R                   : Reset camera
  Left click          : Select person
  ESC                 : Quit
"""

import sys, math, random, heapq

try:
    import pygame
    from pygame.math import Vector2
except ImportError:
    print("Install pygame: pip install pygame")
    sys.exit(1)

# ---------------------------- Constants ----------------------------

WORLD_W, WORLD_H = 3000, 2000
SCREEN_W, SCREEN_H = 1024, 700
FPS = 60

NUM_TREES = 3500
NUM_PEOPLE = 20
TREE_MIN, TREE_MAX = 8, 26
ROAD_WIDTH = 28
EXIT_POS = Vector2(WORLD_W - 150, WORLD_H // 2)
EXIT_RADIUS = 40
NODE_SPACING = 150
CONNECTION_RADIUS = 220
OBSTACLE_PROB = 0.08  # 8% chance road segment has an obstacle

GRID_SIZE = 40  # for spatial bins

# ---------------------------- Camera ----------------------------

class Camera:
    def __init__(self, screen_size, world_size):
        self.sw, self.sh = screen_size
        self.ww, self.wh = world_size
        self.offset = Vector2((self.ww - self.sw)//2, (self.wh - self.sh)//2)
        self.zoom = 1.0
        self.follow = True
    def world_to_screen(self,pos): return (pos - self.offset) * self.zoom
    def screen_to_world(self,pos): return Vector2(pos) / self.zoom + self.offset
    def clamp_offset(self):
        self.offset.x = max(0, min(self.offset.x, self.ww - self.sw/self.zoom))
        self.offset.y = max(0, min(self.offset.y, self.wh - self.sh/self.zoom))
    def center_on(self,pos):
        self.offset = pos - Vector2(self.sw, self.sh)/(2*self.zoom)
        self.clamp_offset()

# ---------------------- Efficient Forest World ----------------------

def spatial_bins(size=GRID_SIZE):
    return dict()

def add_tree_to_bin(tree_bins, x, y, s):
    gx, gy = int(x // GRID_SIZE), int(y // GRID_SIZE)
    tree_bins.setdefault((gx, gy), []).append((x, y, s))

def is_tree_pos_ok(tree_bins, tx, ty, s):
    gx, gy = int(tx // GRID_SIZE), int(ty // GRID_SIZE)
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            group = tree_bins.get((gx + dx, gy + dy), [])
            for ox, oy, os in group:
                if math.hypot(tx - ox, ty - oy) < (s + os) * 0.8:
                    return False
    return True

def build_graph_network():
    nodes = []
    edges = {}
    margin = 100
    for x in range(margin, WORLD_W - margin, NODE_SPACING):
        for y in range(margin, WORLD_H - margin, NODE_SPACING):
            offset_x = random.randint(-30, 30)
            offset_y = random.randint(-30, 30)
            node = (x + offset_x, y + offset_y)
            nodes.append(node)
            edges[node] = []
    exit_node = (int(EXIT_POS.x), int(EXIT_POS.y))
    nodes.append(exit_node)
    edges[exit_node] = []

    # random connection graph
    for i, node1 in enumerate(nodes):
        for node2 in nodes[i + 1 :]:
            dist = math.hypot(node1[0] - node2[0], node1[1] - node2[1])
            if dist < CONNECTION_RADIUS and random.random() < 0.88:
                edges[node1].append(node2)
                edges[node2].append(node1)
    for node in nodes:
        if node != exit_node:
            dist = math.hypot(node[0] - exit_node[0], node[1] - exit_node[1])
            if dist < CONNECTION_RADIUS * 1.5 and exit_node not in edges[node]:
                edges[node].append(exit_node)
                edges[exit_node].append(node)
    return nodes, edges

def build_world():
    nodes, edges = build_graph_network()
    road_segments = []
    processed = set()
    for node, neighbors in edges.items():
        for neighbor in neighbors:
            edge_key = tuple(sorted([node, neighbor]))
            if edge_key not in processed:
                road_segments.append((Vector2(node), Vector2(neighbor)))
                processed.add(edge_key)
    tree_bins = spatial_bins()
    trees = []
    attempts = 0
    max_attempts = NUM_TREES * 10
    while len(trees) < NUM_TREES and attempts < max_attempts:
        tx = int(random.uniform(100, WORLD_W - 100))
        ty = int(random.uniform(100, WORLD_H - 100))
        s = random.randint(TREE_MIN, TREE_MAX)
        if is_tree_pos_ok(tree_bins, tx, ty, s):
            # Don't place trees on roads
            too_close = False
            for seg_start, seg_end in random.sample(road_segments, min(35, len(road_segments))):
                line_vec = seg_end - seg_start
                point_vec = Vector2(tx, ty) - seg_start
                line_len = line_vec.length()
                if line_len < 0.1: continue
                line_unitvec = line_vec / line_len
                proj_length = max(0, min(line_len, point_vec.dot(line_unitvec)))
                closest = seg_start + line_unitvec * proj_length
                dist_to_road = (Vector2(tx, ty) - closest).length()
                if dist_to_road < ROAD_WIDTH + 10:
                    too_close = True
                    break
            if not too_close:
                trees.append((tx, ty, s))
                add_tree_to_bin(tree_bins, tx, ty, s)
        attempts += 1

    # Create some random obstacles on road segments for demo
    obstacles = set()
    for seg_start, seg_end in random.sample(road_segments, max(1, int(OBSTACLE_PROB * len(road_segments)))):
        obstacles.add(tuple(sorted([tuple(seg_start), tuple(seg_end)])))
    print(f"Created {len(nodes)} road nodes and {len(road_segments)} segments; Placed {len(trees)} trees")
    print(f"Obstacles on {len(obstacles)} road segments")
    return {'nodes': nodes, 'edges': edges, 'trees': trees, 'obstacles': obstacles}

# --------------------------- Drone ----------------------------

class Drone:
    def __init__(self, pos):
        self.pos = Vector2(pos)
        self.speed = 250
        self.r = 14
    def update(self, dt, keys):
        v = Vector2(0,0)
        if keys[pygame.K_w] or keys[pygame.K_UP]: v.y -= 1
        if keys[pygame.K_s] or keys[pygame.K_DOWN]: v.y += 1
        if keys[pygame.K_a] or keys[pygame.K_LEFT]: v.x -= 1
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]: v.x += 1
        if v.length_squared()>0:
            v = v.normalize()*self.speed*dt
            self.pos += v
            self.pos.x = max(0,min(self.pos.x,WORLD_W))
            self.pos.y = max(0,min(self.pos.y,WORLD_H))

# ----------------- A* Pathfinding w/ Obstacles ------------------

def a_star(start, goal, edges, obstacles):
    if start not in edges or goal not in edges:
        return []
    frontier = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal: break
        for next_node in edges[current]:
            edge_key = tuple(sorted([current, next_node]))
            if edge_key in obstacles: continue
            new_cost = cost_so_far[current] + math.hypot(next_node[0] - current[0], next_node[1] - current[1])
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + math.hypot(goal[0] - next_node[0], goal[1] - next_node[1])
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
    if goal not in came_from:
        return []
    path = []
    node = goal
    while node is not None:
        path.append(Vector2(node))
        if node == start: break
        node = came_from.get(node)
    path.reverse()
    return path

# -------------------------- Person Agent --------------------------

class Person:
    def __init__(self, unique_id, world):
        node = random.choice(world['nodes'])
        offset_x = random.randint(-50, 50)
        offset_y = random.randint(-50, 50)
        self.pos = Vector2(node[0] + offset_x, node[1] + offset_y)
        self.speed = 50
        self.path = []
        self.color = (70, 100, 220)
        self.selected = False
        self.world = world
    def step(self):
        if self.path:
            target = self.path[0]
            v = target - self.pos
            if v.length() > 1:
                self.pos += v.normalize() * self.speed * (1/FPS)
            else:
                self.pos = target
                self.path.pop(0)
                if not self.path:
                    self.selected = False
    def go_to_exit(self):
        self.selected = True
        self.color = (220, 70, 100)
        nearest = min(self.world['nodes'], key=lambda n: (Vector2(n) - self.pos).length())
        exit_node = min(self.world['nodes'], key=lambda n: (Vector2(n) - EXIT_POS).length())
        path = a_star(nearest, exit_node, self.world['edges'], self.world['obstacles'])
        if path:
            self.path = [self.pos] + path + [EXIT_POS]
        else:
            self.selected = False
            self.color = (255, 40, 40)

# -------------------------- Drawing -------------------------

def draw_world(surf, world, cam, people):
    surf.fill((100, 135, 95))
    # Roads
    drawn_edges = set()
    for node, neighbors in world['edges'].items():
        a = cam.world_to_screen(Vector2(node))
        for neighbor in neighbors:
            edge_key = tuple(sorted([node, neighbor]))
            if edge_key not in drawn_edges:
                b = cam.world_to_screen(Vector2(neighbor))
                width = max(2, int(ROAD_WIDTH * 0.5 * cam.zoom))
                color = (200, 60, 60) if edge_key in world['obstacles'] else (150, 120, 90)
                pygame.draw.line(surf, color, (int(a.x), int(a.y)), (int(b.x), int(b.y)), width)
                drawn_edges.add(edge_key)
    # Nodes
    for node in world['nodes']:
        nc = cam.world_to_screen(Vector2(node))
        if -50 < nc.x < cam.sw + 50 and -50 < nc.y < cam.sh + 50:
            r = max(2, int(4 * cam.zoom))
            pygame.draw.circle(surf, (120, 100, 70), (int(nc.x), int(nc.y)), r)
    # Trees
    for tx, ty, s in world['trees']:
        sc = cam.world_to_screen(Vector2(tx, ty))
        r = max(1, int(s * cam.zoom))
        if -50 < sc.x < cam.sw + 50 and -50 < sc.y < cam.sh + 50:
            pygame.draw.circle(surf, (20, 20, 20), (int(sc.x + r * 0.3), int(sc.y + r * 0.3)), int(r * 1.1))
            g = min(255, 120 + (s % 100))
            pygame.draw.circle(surf, (30, g, 20), (int(sc.x), int(sc.y)), r)
    # Exit
    ex = cam.world_to_screen(EXIT_POS)
    size = int(EXIT_RADIUS * cam.zoom)
    pygame.draw.circle(surf, (255, 100, 100), (int(ex.x), int(ex.y)), size)
    pygame.draw.circle(surf, (255, 0, 0), (int(ex.x), int(ex.y)), int(size * 0.7))
    # People
    for p in people:
        if p.selected and len(p.path) > 1:
            path_points = [(int(cam.world_to_screen(pos).x), int(cam.world_to_screen(pos).y)) for pos in p.path]
            if len(path_points) > 1:
                pygame.draw.lines(surf, (255, 200, 100), False, path_points, max(1, int(3 * cam.zoom)))
        ps = cam.world_to_screen(p.pos)
        r = max(3, int(6 * cam.zoom))
        pygame.draw.circle(surf, (0, 0, 0), (int(ps.x), int(ps.y)), r + 1)
        pygame.draw.circle(surf, p.color, (int(ps.x), int(ps.y)), r)

# ---------------------------- Main ------------------------------

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Forest Arena - Efficient Graph Road Network")
    clock = pygame.time.Clock()
    print("Building world...")
    world = build_world()
    cam = Camera((SCREEN_W, SCREEN_H), (WORLD_W, WORLD_H))
    drone = Drone((WORLD_W/2, WORLD_H/2))
    people = [Person(i, world) for i in range(NUM_PEOPLE)]
    dragging, last = False, None
    running = True
    font = pygame.font.Font(None, 24)

    while running:
        dt = clock.tick(FPS) / 1000
        keys = pygame.key.get_pressed()
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_SPACE:
                    cam.follow = not cam.follow
                if e.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    cam.zoom = min(3, cam.zoom * 1.1)
                if e.key in (pygame.K_MINUS, pygame.K_UNDERSCORE):
                    cam.zoom = max(0.3, cam.zoom / 1.1)
                if e.key == pygame.K_r:
                    cam.zoom = 1
                    cam.offset = Vector2((WORLD_W - SCREEN_W)//2, (WORLD_H - SCREEN_H)//2)
            elif e.type == pygame.MOUSEBUTTONDOWN:
                if e.button == 1:
                    dragging = True
                    last = Vector2(e.pos)
                    world_pos = cam.screen_to_world(e.pos)
                    for p in people:
                        if (p.pos - world_pos).length() < 20:
                            p.go_to_exit()
                            break
                if e.button == 4: cam.zoom = min(3, cam.zoom * 1.1)
                if e.button == 5: cam.zoom = max(0.3, cam.zoom / 1.1)
            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                dragging = False
            elif e.type == pygame.MOUSEMOTION and dragging:
                diff = Vector2(e.pos) - last
                cam.offset -= diff / cam.zoom
                last = Vector2(e.pos)
                cam.clamp_offset()

        drone.update(dt, keys)
        if cam.follow: cam.center_on(drone.pos)
        for p in people: p.step()
        draw_world(screen, world, cam, people)
        dpos = cam.world_to_screen(drone.pos)
        dr = max(4, int(drone.r * cam.zoom))
        pygame.draw.circle(screen, (50, 50, 50), (int(dpos.x), int(dpos.y)), dr + 2)
        pygame.draw.circle(screen, (200, 50, 50), (int(dpos.x), int(dpos.y)), dr)
        pygame.draw.circle(screen, (255, 150, 150), (int(dpos.x - dr*0.3), int(dpos.y - dr*0.3)), dr // 3)
        # UI
        ui_texts = [
            f"Zoom: {cam.zoom:.1f}x",
            f"Follow: {'ON' if cam.follow else 'OFF'}",
            "Click person to send to exit"
        ]
        for i, text in enumerate(ui_texts):
            surf = font.render(text, True, (255, 255, 255))
            shadow = font.render(text, True, (0, 0, 0))
            screen.blit(shadow, (11, 11 + i*25))
            screen.blit(surf, (10, 10 + i*25))
        pygame.display.flip()
    pygame.quit()

if __name__ == "__main__":
    main()
