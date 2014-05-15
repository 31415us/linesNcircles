
from Vec2D import Vec2D
from Circle import Circle,Obstacle

ROBOT_MAX_ACC = 1.6 # m/s^2
ROBOT_MAX_V = 0.6 # m/s
ROBOT_RADIUS = 0.2 # m
ROBOT_DIAMETER = 2 * ROBOT_RADIUS # m

MIN_R = ((ROBOT_MAX_V * ROBOT_MAX_V) / ROBOT_MAX_ACC)

PLAYGROUND_BORDER = [Vec2D(0,0),Vec2D(3,0),Vec2D(3,2),Vec2D(0,2)]

PLAYGROUND_WIDTH = 3
PLAYGROUND_HEIGHT = 2

DELTA_T = 0.1 # s

MIDDLE_CIRCLE = Obstacle(Circle(Vec2D(1.5,1.05),0.15 + ROBOT_RADIUS),Vec2D(0,0))
TREE1 = Obstacle(Circle(Vec2D(0,1.3),0.12 + ROBOT_RADIUS),Vec2D(0,0))
TREE2 = Obstacle(Circle(Vec2D(0.7,2.0),0.12 + ROBOT_RADIUS),Vec2D(0,0))
TREE3 = Obstacle(Circle(Vec2D(2.3,2.0),0.12 + ROBOT_RADIUS),Vec2D(0,0))
TREE4 = Obstacle(Circle(Vec2D(3,1.3),0.12 + ROBOT_RADIUS),Vec2D(0,0))

STATIC_OBSTACLES = [MIDDLE_CIRCLE,TREE1,TREE2,TREE3,TREE4]

def check_playground(pt):
    return pt.x >= 0 and pt.x <= PLAYGROUND_WIDTH and pt.y >= 0 and pt.y <= PLAYGROUND_HEIGHT

