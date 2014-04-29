
from Vec2D import Vec2D

ROBOT_MAX_ACC = 1.6 # m/s^2
ROBOT_MAX_V = 0.6 # m/s
ROBOT_RADIUS = 0.2 # m
ROBOT_DIAMETER = 2 * ROBOT_RADIUS # m

MIN_R = ((ROBOT_MAX_V * ROBOT_MAX_V) / ROBOT_MAX_ACC)

PLAYGROUND_BORDER = [Vec2D(0,0),Vec2D(3,0),Vec2D(3,2),Vec2D(0,2)]

PLAYGROUND_WIDTH = 3
PLAYGROUND_HEIGHT = 2

DELTA_T = 0.1 # s

def check_playground(pt):
    return pt.x >= 0 and pt.x <= PLAYGROUND_WIDTH and pt.y >= 0 and pt.y <= PLAYGROUND_HEIGHT

