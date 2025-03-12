import pygame
import math
import numpy as np
from pygame.locals import *

# Initialize pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 600
FPS = 60
BACKGROUND_COLOR = (0, 0, 0)
HEXAGON_COLOR = (0, 150, 255)
BALL_COLOR = (255, 100, 100)
HEXAGON_RADIUS = 200
BALL_RADIUS = 15
GRAVITY = 0.5
FRICTION = 0.95
ROTATION_SPEED = 0.01  # Radians per frame

# Setup the display
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Ball Bouncing in Spinning Hexagon")
clock = pygame.time.Clock()

# Ball properties
ball_pos = np.array([WIDTH/2, HEIGHT/2], dtype=float)
ball_vel = np.array([3.0, 1.0], dtype=float)

# Hexagon properties
hexagon_center = np.array([WIDTH/2, HEIGHT/2])
hexagon_angle = 0

def create_hexagon(center, radius, angle):
    """Create hexagon vertices based on center, radius, and rotation angle."""
    vertices = []
    for i in range(6):
        theta = angle + i * (2 * math.pi / 6)
        x = center[0] + radius * math.cos(theta)
        y = center[1] + radius * math.sin(theta)
        vertices.append((x, y))
    return vertices

def distance_point_to_line(point, line_start, line_end):
    """Calculate the distance from a point to a line segment."""
    line_vec = line_end - line_start
    point_vec = point - line_start
    line_len = np.linalg.norm(line_vec)
    line_unitvec = line_vec / line_len
    point_vec_scaled = point_vec / line_len
    t = np.clip(np.dot(line_unitvec, point_vec_scaled), 0, 1)
    nearest = line_start + t * line_vec
    dist = np.linalg.norm(nearest - point)
    return dist, nearest, t

def reflect_velocity(velocity, normal):
    """Reflect velocity vector across a normal vector."""
    normal = normal / np.linalg.norm(normal)  # Ensure normal is unit vector
    return velocity - 2 * np.dot(velocity, normal) * normal

def handle_collision(ball_pos, ball_vel, vertices):
    """Handle collision between ball and hexagon sides."""
    for i in range(6):
        line_start = np.array(vertices[i])
        line_end = np.array(vertices[(i + 1) % 6])
        
        # Calculate distance from ball to line segment
        dist, nearest, t = distance_point_to_line(ball_pos, line_start, line_end)
        
        # Check for collision
        if dist < BALL_RADIUS:
            # Calculate the normal vector to the line
            line_vec = line_end - line_start
            normal = np.array([-line_vec[1], line_vec[0]])
            normal = normal / np.linalg.norm(normal)
            
            # Make sure normal points towards the ball
            if np.dot(normal, ball_pos - nearest) < 0:
                normal = -normal
            
            # Move ball outside the collision
            penetration = BALL_RADIUS - dist
            ball_pos += normal * penetration
            
            # Calculate relative velocity of the wall at collision point
            wall_vel = np.array([0.0, 0.0])
            if 0 <= t <= 1:  # If collision is on the line segment
                # Calculate tangential velocity of the wall due to rotation
                radius_vector = nearest - hexagon_center
                tangent = np.array([-radius_vector[1], radius_vector[0]])
                tangent = tangent / np.linalg.norm(tangent)
                wall_vel = tangent * ROTATION_SPEED * np.linalg.norm(radius_vector)
            
            # Adjust ball velocity (include wall velocity in calculation)
            rel_vel = ball_vel - wall_vel
            
            # Apply reflection with friction
            ball_vel = reflect_velocity(rel_vel, normal) * FRICTION + wall_vel
            
            return ball_pos, ball_vel, True
            
    return ball_pos, ball_vel, False

# Main game loop
running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
    
    # Update hexagon rotation
    hexagon_angle += ROTATION_SPEED
    vertices = create_hexagon(hexagon_center, HEXAGON_RADIUS, hexagon_angle)
    
    # Apply gravity to ball velocity
    ball_vel[1] += GRAVITY
    
    # Update ball position
    ball_pos += ball_vel
    
    # Handle collisions with hexagon
    collision_detected = True
    max_iterations = 5  # Prevent infinite loops
    iterations = 0
    
    while collision_detected and iterations < max_iterations:
        ball_pos, ball_vel, collision_detected = handle_collision(ball_pos, ball_vel, vertices)
        iterations += 1
    
    # Draw everything
    screen.fill(BACKGROUND_COLOR)
    
    # Draw hexagon
    pygame.draw.polygon(screen, HEXAGON_COLOR, vertices, 2)
    
    # Draw ball
    pygame.draw.circle(screen, BALL_COLOR, (int(ball_pos[0]), int(ball_pos[1])), BALL_RADIUS)
    
    # Update display
    pygame.display.flip()
    
    # Cap the frame rate
    clock.tick(FPS)

pygame.quit()
