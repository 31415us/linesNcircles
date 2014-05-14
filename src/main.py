
import pygame,sys
import json
import socket

from Circle import Circle
from Vec2D import Vec2D
import Globals
from Environment import Environment,Obstacle

import json
import socket

#pygame setup and vars
px_per_meter = 400 
width = Globals.PLAYGROUND_WIDTH * px_per_meter
height = Globals.PLAYGROUND_HEIGHT * px_per_meter
background = pygame.image.load("../demo/background.png")
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

    c5 = Circle(Vec2D(1.5,1),0.3)
    c6 = Circle(Vec2D(0.75,0.5),0.3)

    null = Vec2D(0,0)

    circles = [c1,c2,c3,c4]

    start = Vec2D(0.01,0.01)
    end = Vec2D(2.99,1.99)

    obstacles = []


    #clock = pygame.time.Clock()

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
            screen.blit(background,(0,0))

            s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            s.connect(('127.0.0.1',1337))
            s.send(json.dumps(to_request(start,null,end,null,0.1,1000,obstacles)))

            traj = traj_from_response(json.loads(s.recv(4096).strip()))

            s.close()

            draw_path(traj)


            #print clock.tick()

            pygame.display.update()

def to_request(start,v_start,end,v_end,delta_t,n,obstacles):
    start_list = [int(start.x*1000),int(start.y*1000),int(v_start.x*1000),int(v_start.y*1000)]
    end_list = [int(end.x*1000),int(end.y*1000)]
    dt = int(delta_t*1000)
    N = n
    obstacle_list = []
    for o in obstacles:
        x = o.circle.pos.x
        y = o.circle.pos.y
        vx = o.v.x
        vy = o.v.y
        r = o.circle.r
        obstacle_list.append([int(x*1000),int(x*1000),int(vx*1000),int(vy*1000),int(r*1000)])
    return [start_list,end_list,dt,N,obstacle_list]

def traj_from_response(response):
    traj = []
    for w in response:
        x = float(w[0])/1000
        y = float(w[1])/1000
        vx = float(w[2])/1000
        vy = float(w[3])/1000
        t = float(w[4])/1000
        traj.append((Vec2D(x,y),Vec2D(vx,vy),t))
    return traj


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

    prev = path[0]

    for curr in path[1:]:
        pygame.draw.line(screen,blue,(int(prev[0].x * px_per_meter),int(prev[0].y*px_per_meter)),(int(curr[0].x*px_per_meter),int(curr[0].y*px_per_meter)),1)
        prev = curr

    #for seg in path:
    #    #print seg
    #    start = seg.start
    #    end = seg.end
    #    pygame.draw.line(screen,blue,(int(start.x * px_per_meter),int(start.y*px_per_meter)),(int(end.x*px_per_meter),int(end.y*px_per_meter)),1)
    #print "\n"



if __name__ == "__main__":
    main()
