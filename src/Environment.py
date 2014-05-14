
from Vec2D import Vec2D,orientation

from Circle import Circle,LineSegment,CircleSegment,discretize

import Globals

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
            if Globals.check_playground(t.p1) and Globals.check_playground(t.p2):
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
            if Globals.check_playground(t.p1) and Globals.check_playground(t.p2):
                result.append(t)

        return result

    def intersects_any(self,segment):
        for o in self.obstacles:
            if o.circle.intersect_segment(segment):
                return True

        return False

    def path(self,start,v_start,end,v_end,delta_t):

        if not self.intersects_any(LineSegment(start,end)):
            return discretize_trajectory([LineSegment(start,end)],Vec2D(0,0),Vec2D(0,0),0.1)

        tans = self.all_tangents()
        circle_map = {}
        neighbours = {}
        orientation_map = {}

        for t in tans:
            if circle_map.get(t.c1):
                circle_map.get(t.c1).add(t.p1)
            else:
                circle_map[t.c1] = set([t.p1])

            if circle_map.get(t.c2):
                circle_map.get(t.c2).add(t.p2)
            else:
                circle_map[t.c2] = set([t.p2])

        for t in tans:
            orientation_map[t.p1] = t.orient1
            orientation_map[t.p2] = t.orient2

        for t in tans:
            p1n = neighbours.get(t.p1) or set()
            p1n = p1n | circle_map[t.c1]
            p1n = p1n - set([t.p1])
            p1n = {x for x in p1n if not same_orientation(t.orient1,orientation_map.get(x))}
            p1n.add(t.p2)
            neighbours[t.p1] = p1n

            p2n = neighbours.get(t.p2) or set()
            p2n = p1n | circle_map[t.c2]
            p2n = p1n - set([t.p2])
            p2n = {x for x in p2n if not same_orientation(t.orient2,orientation_map.get(x))}
            p2n.add(t.p1)
            neighbours[t.p2] = p2n

        neighbours[start] = set()
        for t in self.tangents_to_point(start):
            orientation_map[t.p1] = t.orient1
            orientation_map[t.p2] = t.orient2
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
            if not circle_map.get(c):
                circle_map[c] = set()
            for point in circle_map[c]:
                neighbours[point].add(p)
            neighbours[p] = neighbours[p] | circle_map[c]
            circle_map[c].add(p)

        neighbours[end] = set()
        for t in self.tangents_to_point(end):
            orientation_map[t.p1] = t.orient1
            orientation_map[t.p2] = t.orient2
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
            if not circle_map.get(c):
                circle_map[c] = set()
            for point in circle_map[c]:
                neighbours[point].add(p)
            neighbours[p] = neighbours[p] | circle_map[c]
            circle_map[c].add(p)

        nodes_on_path = aStar(start,end,circle_map,neighbours)
        node_pairs = zip(nodes_on_path[:-1],nodes_on_path[1:])

        segs = []
        for p in node_pairs:
            s_seg = p[0]
            e_seg = p[1]
            c = same_circle(circle_map,s_seg,e_seg)
            if c is None:
                segs.append(LineSegment(s_seg,e_seg))
            else:
                segs.append(CircleSegment(s_seg,e_seg,c,orientation_map[s_seg]))

        return discretize_trajectory(segs,v_start,v_end,delta_t)

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
            
            res.append(curr)

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

def discretize_trajectory(segments,v_start,v_end,delta_t):
    errors = []
    vs = v_start.dot((segments[0].start - segments[0].end).normalized())
    ve = v_end.length()
    traj_length = 0
    for seg in segments:
        traj_length = traj_length + seg.length()

    acc_until_dist = 0
    decc_from_dist = 0

    t_adj = abs(vs - ve)/Globals.ROBOT_MAX_ACC
    d_adj = min(vs,ve)*t_adj + 0.5 * Globals.ROBOT_MAX_ACC * t_adj * t_adj

    t_acc = (Globals.ROBOT_MAX_V - vs)/Globals.ROBOT_MAX_ACC
    d_acc = vs * t_acc + 0.5 * Globals.ROBOT_MAX_ACC * t_acc * t_acc
    
    t_dec = (Globals.ROBOT_MAX_V - ve)/Globals.ROBOT_MAX_ACC
    d_decc = ve * t_dec + 0.5 * Globals.ROBOT_MAX_ACC * t_dec * t_dec

    if d_adj > traj_length :
        if vs < ve :
            acc_until_dist = traj_length
            decc_from_dist = traj_length + 1
            errors.append(Err("Cannot accelerate to end-velocity"))
        else:
            acc_until_dist = 0
            decc_from_dist = traj_length
            errors.append(Err("Cannot decelerate to end-velocity"))
    elif t_acc + t_dec > traj_length:
        d_tmp = (traj_length - d_adj)/2
        acc_until_dist = d_tmp
        decc_from_dist = d_tmp
    else:
        acc_until_dist = d_acc
        decc_from_dist = traj_length - d_decc

    current_d = 0
    current_t = 0 
    current_v = vs
    res = []
    for seg in segments:
        res = res + discretize(seg,current_d,current_t,current_v,acc_until_dist,decc_from_dist,delta_t)
        (pos,v,time_stamp) = res[-1]
        current_d = current_d + seg.length()
        current_t = time_stamp
        current_v = v.length()

    return res

class Err(object):

    def __init__(self,msg):
        self.msg = msg

    def __str__(self):
        return self.msg
