
import pygame,sys

from Circle import Circle
from Vec2D import Vec2D
import Globals
from Environment import Environment,Obstacle

#pygame setup and vars
px_per_meter = 400 
width = Globals.PLAYGROUND_WIDTH * px_per_meter
height = Globals.PLAYGROUND_HEIGHT * px_per_meter
pygame.init()
screen = pygame.display.set_mode((width,height))
white = (255,255,255)
black = (0,0,0)
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)

def main():

    c1 = Circle(Vec2D(1.5,1),0.3)
    c2 = Circle(Vec2D(0.4,1.3),0.3)
    c3 = Circle(Vec2D(0.2,0.22),0.1)
    c4 = Circle(Vec2D(0.4,0.6),0.2)

    circles = [c1,c2,c3,c4]

    env = Environment([Obstacle(c1,Vec2D(-0.0001,0)),Obstacle(c2,Vec2D(0,-0.0001)),Obstacle(c3,Vec2D(0.0001,0)),Obstacle(c4,Vec2D(0,0.0001))])

    #env = Environment([Obstacle(c1,Vec2D(0.0000,0.0001)),Obstacle(c4,Vec2D(0,-0.0001))])
    #env = Environment([Obstacle(c1,Vec2D(0,-0.0001))])

    start = Vec2D(0.01,0.01)
    end = Vec2D(2.99,1.99)

    path = env.path(start,end)

    clock = pygame.time.Clock()

    paused = False
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    paused = not paused

        if not paused:

            screen.fill(black)

            env.update()
            draw_env(env,start,end)
            draw_path(env.path(start,end))

            #print clock.tick()

            pygame.display.update()


def draw_circle(c):
    pygame.draw.circle(screen,green,(int(c.pos.x*px_per_meter),int(c.pos.y*px_per_meter)),int(c.r*px_per_meter),1)

def draw_line(l):
    pygame.draw.line(screen,red,(int(l[0].x * px_per_meter),int(l[0].y*px_per_meter)),(int(l[1].x*px_per_meter),int(l[1].y*px_per_meter)),1)

def draw_segment(s):
    draw_line((s.start,s.end))

def draw_tangent(t):
    draw_segment(t.segment)

def draw_env(env,start,end):
    for o in env.obstacles:
        draw_circle(o.circle)

    for t in env.all_tangents():
        draw_tangent(t)

    for t in env.tangents_to_point(start):
        draw_tangent(t)

    for t in env.tangents_to_point(end):
        draw_tangent(t)

def draw_path(path):
    #for p in path:
    #    print p
    #print ""
    for i in range(0,len(path) - 1):
        pygame.draw.line(screen,blue,(int(path[i].x * px_per_meter),int(path[i].y*px_per_meter)),(int(path[i+1].x*px_per_meter),int(path[i+1].y*px_per_meter)),1)



if __name__ == "__main__":
    main()
