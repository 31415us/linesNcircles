
from math import sqrt,acos,pi

from Vec2D import Vec2D,orientation

class Circle(object):

    def __init__(self,pos,r):
        self.pos = pos
        self.r = r

    def intersects(self,other):
        return (self.pos - other.pos).length() < (self.r + other.r)

    def contains_circle(self,other):
        return (self.pos - other.pos).length() < (self.r - other.r)

    def intersect_segment(self,segment):

        d = segment.end - segment.start
        oc = segment.start - self.pos

        a = d.dot(d)
        b = 2 * d.dot(oc)
        c = oc.dot(oc) - (self.r * self.r)

        for t in solve_quadratic(a,b,c):
            if t >= 0 and t <= 1:
                return True

        return False

    def bitangents(self,other):
        if self.contains_circle(other) or other.contains_circle(self):
            return []

        l = self.outertangents(other)

        if not self.intersects(other):
            m = self.innertangents(other)

            for v in m:
                l.append(v)

        return l

    def contains_point(self,point):
        return (self.pos - point).length() <= self.r

    def tangents(self,point):
        if not self.contains_point(point):
            return self.outertangents(Circle(point,0))
        else:
            return []


    def innertangents(self,other):
        return self.circle_tangents(other,True)

    def outertangents(self,other):
        return self.circle_tangents(other,False)

    def circle_tangents(self,other,invert_second_radius):
        dx = other.pos.x - self.pos.x
        dy = other.pos.y - self.pos.y
        if not invert_second_radius:
            dr = other.r - self.r
        else:
            dr = - other.r - self.r
        d = sqrt(dx*dx + dy*dy)

        x = dx/d
        y = dy/d
        r = dr/d

        a1 = r*x - y*sqrt(1 - r*r)
        b1 = r*y + x*sqrt(1 - r*r)
        c1 = self.r - (a1*self.pos.x + b1*self.pos.y)

        a2 = r*x + y*sqrt(1 - r*r)
        b2 = r*y - x*sqrt(1 - r*r)
        c2 = self.r - (a2*self.pos.x + b2*self.pos.y)

        x11 = (b1*(b1*self.pos.x - a1*self.pos.y) - a1*c1)
        y11 = (a1*(-b1*self.pos.x + a1*self.pos.y) - b1*c1)

        x12 = (b1*(b1*other.pos.x - a1*other.pos.y) - a1*c1)
        y12 = (a1*(-b1*other.pos.x + a1*other.pos.y) - b1*c1)

        x21 = (b2*(b2*self.pos.x - a2*self.pos.y) - a2*c2)
        y21 = (a2*(-b2*self.pos.x + a2*self.pos.y) - b2*c2)

        x22 = (b2*(b2*other.pos.x - a2*other.pos.y) - a2*c2)
        y22 = (a2*(-b2*other.pos.x + a2*other.pos.y) - b2*c2)

        start1 = Vec2D(x11,y11)
        end1 = Vec2D(x12,y12)
        orient11 = orientation(end1,start1,self.pos) 
        orient12 = orientation(start1,end1,other.pos)

        start2 = Vec2D(x21,y21)
        end2 = Vec2D(x22,y22)
        orient21 = orientation(end2,start2,self.pos) 
        orient22 = orientation(start2,end2,other.pos)
        
        tan1 = Tangent(start1,self,orient11,end1,other,orient12)
        tan2 = Tangent(start2,self,orient21,end2,other,orient22)
        return [tan1,tan2]

    def __str__(self):
        return "Circle: {pos}, {r}".format(pos=str(self.pos),r=self.r)

    def __add__(self,v):
        return Circle(self.pos + v,self.r)

    def __eq__(self,other):
        return (self.pos == other.pos and abs(self.r - other.r) < Vec2D.EPSILON)

    def __ne__(self,other):
        return not self == other

    def tangent_vector(self,p,orientation):
        vr = (p - self.pos).normalized()

        if orientation < 0:
            return Vec2D(-vr.y,vr.x)
        else:
            return Vec2D(vr.y,-vr.x)

class LineSegment(object):
    
    def __init__(self,start,end):
        self.start = start
        self.end = end

    def __str__(self):
        return "LineSegment: {start} -> {end}".format(start=(str(self.start)),end=(str(self.end)))

    def length(self):
        return (self.end - self.start).length()

    def tan(self,pt):
        return (self.end - self.start).normalized()

    def next_pos(self,pt,dist):
        return pt + self.tan(pt) * dist


class CircleSegment(object):

    def __init__(self,start,end,circle,orientation):
        self.start = start
        self.end = end
        self.circle = circle
        self.orientation = orientation

    def __str__(self):
        return "CircleSegment: {start} -> {end} on {circle}".format(start=(str(self.start)),end=(str(self.end)),circle=(str(self.circle)))

    def length(self):
        apparent_orientation = orientation(self.start,self.end,self.circle.pos)

        v1 = (self.start - self.circle.pos).normalized()
        v2 = (self.end - self.circle.pos).normalized()

        angle = acos(v1.dot(v2))

        if self.orientation * apparent_orientation < 0:
            angle = 2*pi - angle

        return angle * self.circle.r

    def tan(self,pt):
        return self.circle.tangent_vector(pt,self.orientation).normalized()

    def next_pos(self,pt,dist):
        angle = (dist / self.circle.r) 
        if self.orientation < 0:
            angle = -angle
        return pt.rotate(angle,self.circle.pos)
        
class Tangent(object):
    
    def __init__(self,p1,c1,orient1,p2,c2,orient2):
        self.p1 = p1
        self.p2 = p2
        self.orient1 = orient1
        self.c1 = c1
        self.c2 = c2
        self.orient2 = orient2
        self.segment = LineSegment(self.p1,self.p2)

class Obstacle(object):

    def __init__(self,circle,v):
        self.circle = circle
        self.v = v

    def update(self):
        self.circle = self.circle + self.v

    def __eq__(self,other):
        return (self.circle == other.circle)

def solve_quadratic(a,b,c):
    det = b*b - 4*a*c

    if det < 0:
        return []
    elif det == 0:
        return [-b/(2*a)]
    else:
        sqrtDet = sqrt(det)
        t1 = (-b + sqrtDet)/(2 * a)
        t2 = (-b - sqrtDet)/(2 * a)
        return [t1, t2]

