
import SocketServer
import json

import Globals

from Vec2D import Vec2D
from Environment import Environment,Obstacle
from Circle import Circle

middle_circle = Obstacle(Circle(Vec2D(1.5,0.95),0.15 + Globals.ROBOT_RADIUS),Vec2D(0,0))
tree1 = Obstacle(Circle(Vec2D(0,1.3),0.12 + Globals.ROBOT_RADIUS),Vec2D(0,0))
tree2 = Obstacle(Circle(Vec2D(0.7,2.0),0.12 + Globals.ROBOT_RADIUS),Vec2D(0,0))
tree3 = Obstacle(Circle(Vec2D(2.3,2.0),0.12 + Globals.ROBOT_RADIUS),Vec2D(0,0))
tree4 = Obstacle(Circle(Vec2D(3,1.3),0.12 + Globals.ROBOT_RADIUS),Vec2D(0,0))

static_obstacles = [middle_circle,tree1,tree2,tree3,tree4]

def trajectory(start,v_start,end,v_end,delta_t,obstacles):
    obs = obstacles + static_obstacles
    env = Environment(obs)
    return env.path(start,v_start,end,v_end,delta_t)

class TrajectoryServer(SocketServer.ThreadingTCPServer):
    allow_reuse_address = True

class TrajectoryHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        try:
            data = json.loads(self.request.recv(2048).strip())

            start_info = data[0]
            end_info = data[1]
            delta_t = float(data[2])/1000 #ms -> s
            nb_of_waypoints = data[3]
            obstacles = data[4]

            start = Vec2D(float(start_info[0])/1000,float(start_info[1])/1000)
            v_start = Vec2D(float(start_info[2])/1000,float(start_info[3])/1000)
            end = Vec2D(float(end_info[0])/1000,float(end_info[1])/1000)
            v_end = Vec2D(0,0)

            obstacle_list = []
            for o in obstacles:
                pos = Vec2D(float(o[0])/1000,float(o[1])/1000)
                speed = Vec2D(float(o[2])/1000,float(o[3])/1000)
                r = float(o[4])/1000
                obstacle_list.append(Obstacle(Circle(pos,r + Globals.ROBOT_RADIUS),speed))

            traj = trajectory(start,v_start,end,v_end,delta_t,obstacle_list)[:nb_of_waypoints]

            response = []
            for w in traj:
                pos = w[0] 
                x = int(pos.x * 1000)
                y = int(pos.y * 1000)
                v = w[1]
                vx = int(v.x * 1000)
                vy = int(v.y * 1000)
                t = int(w[2] * 1000)
                response.append([x,y,vx,vy,t])

            self.request.sendall(json.dumps(response).strip())
        except Exception, e:
            print "Exception wtf?", e


server = TrajectoryServer(('0.0.0.0',1337),TrajectoryHandler)
server.serve_forever()
