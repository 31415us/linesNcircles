
from Vec2D import Vec2D,orientation

from Circle import Circle,LineSegment

from Globals import check_playground

from heapdict import heapdict

from math import acos,pi

class Obstacle(object):

    def __init__(self,circle,v):
        self.circle = circle
        self.v = v

    def update(self):
        self.circle = self.circle + self.v

    def __eq__(self,other):
        return (self.circle == other.circle)

class Environment(object):

    def __init__(self,obstacles):
        self.obstacles = obstacles

    def update(self):
        for o in self.obstacles:
            o.update()

    def vmap(self):
        return {o.circle : o.v for o in self.obstacles}

    def all_tangents(self):
        if len(self.obstacles) <= 1:
            return []

        tangents = []
        circles = [o.circle for o in self.obstacles]
        circlePairs = [(circles[i],c) for i in range(0,len(circles)) for c in circles[i+1:]]
        for pair in circlePairs:
            other_circles = [o.circle for o in self.obstacles if not o.circle == pair[0] and not o.circle == pair[1]]
            tans = pair[0].bitangents(pair[1])
            for t in tans:
                intersection = False
                for c in other_circles:
                    if c.intersect_segment(t.segment):
                        intersection = True

                if not intersection:
                    tangents.append(t)

        result = []                
        for t in tangents:
            if check_playground(t.p1) and check_playground(t.p2):
                result.append(t)
                
        return result

    def tangents_to_point(self,pt):
        acc = []
        for obs in self.obstacles:
            other_obs = [o.circle for o in self.obstacles if not o == obs]
            tans = obs.circle.tangents(pt)
            for t in tans:
                intersection = False
                for c in other_obs:
                    if c.intersect_segment(t.segment):
                        intersection = True

                if not intersection:
                    acc.append(t)

        result = []
        for t in acc:
            if check_playground(t.p1) and check_playground(t.p2):
                result.append(t)

        return result

    def intersects_any(self,segment):
        for o in self.obstacles:
            if o.circle.intersect_segment(segment):
                return True

        return False

    def path(self,start,end):

        if not self.intersects_any(LineSegment(start,end)):
            return [start,end]

        tans = self.all_tangents()
        same_circle = {}
        neighbours = {}
        orientation_map = {}

        for t in tans:
            if same_circle.get(t.c1):
                same_circle.get(t.c1).add(t.p1)
            else:
                same_circle[t.c1] = set([t.p1])

            if same_circle.get(t.c2):
                same_circle.get(t.c2).add(t.p2)
            else:
                same_circle[t.c2] = set([t.p2])

        for t in tans:
            orientation_map[t.p1] = t.orient1
            orientation_map[t.p2] = t.orient2

        # filter out points on the half-circle defined by the moving direction
        #v_map = self.vmap()
        #for c in same_circle:
        #    v = v_map[c]
        #    if v.length() == 0:
        #        continue
        #    v = v.normalized()
        #    to_remove = set()
        #    for p in same_circle[c]:
        #        d = (p - c.pos).normalized()
        #        if d.dot(v) > 0:
        #            to_remove.add(p)
        #    same_circle[c] = same_circle[c] - to_remove

        for t in tans:
            p1n = neighbours.get(t.p1) or set()
            p1n = p1n | same_circle[t.c1]
            p1n = p1n - set([t.p1])
            p1n = {x for x in p1n if not same_orientation(t.orient1,orientation_map.get(x))}
            p1n.add(t.p2)
            neighbours[t.p1] = p1n

            p2n = neighbours.get(t.p2) or set()
            p2n = p1n | same_circle[t.c2]
            p2n = p1n - set([t.p2])
            p2n = {x for x in p2n if not same_orientation(t.orient2,orientation_map.get(x))}
            p2n.add(t.p1)
            neighbours[t.p2] = p2n

        neighbours[start] = set()
        for t in self.tangents_to_point(start):
            if t.p1 == start:
                p = t.p2
                c = t.c2
            elif t.p2 == start:
                p = t.p1
                c = t.c1
            else:
                print "something went horribly wrong!"

            neighbours[start].add(p)
            neighbours[p] = set([start])
            if not same_circle.get(c):
                same_circle[c] = set()
            for point in same_circle[c]:
                neighbours[point].add(p)
            neighbours[p] = neighbours[p] | same_circle[c]
            same_circle[c].add(p)

        neighbours[end] = set()
        for t in self.tangents_to_point(end):
            if t.p1 == end:
                p = t.p2
                c = t.c2
            elif t.p2 == end:
                p = t.p1
                c = t.c1
            else:
                print "something went horribly wrong!"

            neighbours[end].add(p)
            neighbours[p] = set([end])
            if not same_circle.get(c):
                same_circle[c] = set()
            for point in same_circle[c]:
                neighbours[point].add(p)
            neighbours[p] = neighbours[p] | same_circle[c]
            same_circle[c].add(p)

        #for p in neighbours: 
        #    print p
        #    for n in neighbours[p]:
        #        print "\t{m}".format(m = n)

        #print ""

        return aStar(start,end,same_circle,neighbours)

def aStar(start,end,circle_map,neighbours):
    visited = set()
    parents = {}
    queue = heapdict()
    g_score = {}
    f_score = {}

    g_score[start] = 0
    f_score[start] = g_score[start] + (start - end).length()
    queue[start] = f_score[start]

    while queue:
        current = queue.popitem()[0] 
        visited.add(current)

        if current == end:
            res = []
            curr = current
            while parents.get(curr):
                res.append(curr)
                curr = parents[curr]

            res.reverse()
            return res

        for neigh in neighbours[current]:
            tentative_g_score = g_score[current] + edge_length(current,neigh,circle_map,parents)
            tentative_f_score = tentative_g_score + (neigh - end).length()
            if (neigh in visited) and (tentative_f_score >= f_score[neigh]):
                continue

            if (not (neigh in visited)) or (tentative_f_score < f_score[neigh]):
                parents[neigh] = current
                g_score[neigh] = tentative_g_score
                f_score[neigh] = tentative_f_score
                queue[neigh] = f_score[neigh]

    return []

def edge_length(p1,p2,circle_map,parents):

    c = same_circle(circle_map,p1,p2)

    if not c:
        return (p1 - p2).length()

    pred = parents[p1]
    tan = p1
    while same_circle(circle_map,p1,pred):
        tan = pred
        pred = parents[pred]

    parent_orientation = orientation(pred,tan,c.pos)
    circle_orientation = orientation(p1,p2,c.pos)
    same_orientation = parent_orientation * circle_orientation > 0

    v1 = (p1 - c.pos).normalized()
    v2 = (p2 - c.pos).normalized()

    dp = v1.dot(v2)
    
    try:
        angle = acos(dp)
    except:
        return 100.0

    if not same_orientation:
        angle = 2*pi - angle

    return angle * c.r

def same_circle(circle_map,p1,p2):
    for key in circle_map:
        if (p1 in circle_map[key]) and (p2 in circle_map[key]):
            return key

    return None

def same_orientation(o1,o2):
    return o1 * o2 > 0
