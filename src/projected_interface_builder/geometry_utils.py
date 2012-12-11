#!/usr/bin/env python
from itertools import groupby, chain
from pprint import pprint

def points_on_line(p1,p2):
    if p1.x() > p2.x():
        p1,p2 = p2,p1
    delta = p1 - p2
    if delta.x() == 0:
        return [type(p1).__call__(p1.x(),y) for y in range(p1.y(),p2.y())]
    error = 0.0
    deltaerr = abs(delta.y()/delta.x())
    y = p1.y()
    points = []
    for x in range(p1.x(),p2.x()):
        points.append(type(p1).__call__(x,y))
        error += deltaerr
        if error >= 0.5:
            y += 1
            error -= 1
    return points
            
def find_largest_rect(poly):
    by_x = dict()
    by_y = dict()
    
    for k, v in groupby(sorted(list(chain(*poly)), key=lambda p: p.x()), key=lambda p: p.x()):
        by_x[k] = list(v)
    for k, v in groupby(sorted(list(chain(*poly)), key=lambda p: p.y()), key=lambda p: p.y()):
        by_y[k] = list(v)
    
    
    
    # pprint(by_x)
    pprint(by_y)
    
    # find point pairs in the x and y axes
    # pairs = []
    # for edge in poly:
    #     for point in edge:
            
            
if __name__ == '__main__':
    from PySide.QtCore import QPoint
    rect = [QPoint(0,5), QPoint(5,10), QPoint(10,5), QPoint(5,0)]
    lines = [points_on_line(rect[i-1],rect[i]) for i in range(0,len(rect))]
    # pprint(lines)
    find_largest_rect(lines)