import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib import collections as mc
import numpy as np
import math
import queue
import random
from datetime import datetime

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.set_ylim(-1, 11)
    ax.set_xlim(-1, 11)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax

'''
Make a patch for a single pology
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0], xy[1]))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch


'''
Render the problem
'''
def drawProblem(robotStart, robotGoal, polygons):
    _, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)
    plt.show()

'''
Grow a simple RRT
'''
def growSimpleRRT(points):
    newPoints = {}
    adjListMap = {}

    bool = True
    count = 0
    # Your code goes here
    for i in points:
        if len(newPoints) == 0:
            newPoints[i] = points[i]
            newPoints[i+1] = points[i+1]
            adjListMap[i] = [i+1]
            adjListMap[i+1] = [i]
        elif len(newPoints) == 2 and bool:
            bool = False
            continue
        else:
            num = 0
            index = 0
            point1 = 0
            point2 = 0
            d = sys.maxsize                             #distance from random point to nearest point on line
            x = 0
            y = 0
            for j in list(adjListMap):                  # j is the key in adjListMap. j just goes through each point
                for k in adjListMap[j]:                # k goes through every point that j is connected to
                    m1 = (newPoints[k][1]-newPoints[j][1])/(newPoints[k][0]-newPoints[j][0])
                    b1 = newPoints[j][1] - m1*newPoints[j][0]
                    m2 = -1/m1
                    b2 = points[i][1] - m2*points[i][0]
                    newPointX = (b2-b1)/(m1-m2)
                    newPointY = m1*newPointX + b1
                    if not collisionDetection(adjListMap,newPoints,[points[i],(newPointX,newPointY)],j,k,0):
                        if (newPoints[k][0] <= newPointX <= newPoints[j][0] or newPoints[k][0] >= newPointX >= newPoints[j][0]) and (newPoints[k][1] <= newPointY <= newPoints[j][1] or newPoints[k][1] >= newPointY >= newPoints[j][1]):
                            if d > math.sqrt((newPointX-points[i][0])**2+(newPointY-points[i][1])**2):
                                d = math.sqrt((newPointX-points[i][0])**2+(newPointY-points[i][1])**2)
                                x = newPointX
                                y = newPointY
                                point1 = j
                                point2 = k
                                num = 0
                        else:
                            if newPoints[k][0] > newPoints[j][0]:
                                if newPointX > newPoints[k][0]:
                                    if d > math.sqrt((newPoints[k][0]-points[i][0])**2+(newPoints[k][1]-points[i][1])**2):
                                        d = math.sqrt((newPoints[k][0]-points[i][0])**2+(newPoints[k][1]-points[i][1])**2)
                                        index = k
                                        num = 1
                                else:
                                    if d > math.sqrt((newPoints[j][0]-points[i][0])**2+(newPoints[j][1]-points[i][1])**2):
                                        d = math.sqrt((newPoints[j][0]-points[i][0])**2+(newPoints[j][1]-points[i][1])**2)
                                        index = j
                                        num = 1
                            else:
                                if newPointX > newPoints[j][0]:
                                    if d > math.sqrt((newPoints[j][0]-points[i][0])**2+(newPoints[j][1]-points[i][1])**2):
                                        d = math.sqrt((newPoints[j][0]-points[i][0])**2+(newPoints[j][1]-points[i][1])**2)
                                        index = j
                                        num = 1
                                else:
                                    if d > math.sqrt((newPoints[k][0]-points[i][0])**2+(newPoints[k][1]-points[i][1])**2):
                                        d = math.sqrt((newPoints[k][0]-points[i][0])**2+(newPoints[k][1]-points[i][1])**2)
                                        index = k
                                        num = 1

            if newPointX != points[i][0] and newPointY != points[i][1]:
                temp = i+count
                newPoints[i] = points[i]
                if num == 0:
                    count+=1
                    newPoints[len(points)+count] = (x,y)
                    if adjListMap.get(i) == None:
                        adjListMap[i] = [len(points)+count]
                    else:
                        adjListMap[temp].append(len(points)+count)
                    adjListMap[len(points)+count] = [i]
                    adjListMap[len(points)+count].append(point1)
                    adjListMap[len(points)+count].append(point2)
                    newList = adjListMap[point1]
                    newList.remove(point2)
                    newList.append(len(points)+count)
                    adjListMap[point1] = newList
                    newList = adjListMap[point2]
                    newList.remove(point1)
                    newList.append(len(points)+count)
                    adjListMap[point2] = newList
                else:
                    adjListMap[i] = [index]
                    adjListMap[index].append(i)


    return newPoints, adjListMap

def intersectLines(line, tree, points):
    firstline = np.array([line[0][0],line[0][1],line[1][0],line[1][1]]).reshape(2,2)
    for i in tree:
        for j in tree[i]:
            if points[i] == line[0] or points[i] == line[1] or points[j] == line[0] or points[j] == line[1]:
                continue
            line2 = [points[i],points[j]]
            secondline = np.array([line2[0][0],line2[0][1],line2[1][0],line2[1][1]]).reshape(2,2)

            linecodes = [Path.MOVETO,Path.LINETO,Path.CLOSEPOLY]
            line_path = Path(firstline)
            line_path2 = Path(secondline)

            if(Path.intersects_path(line_path,line_path2,filled=True)):
                return True
    return False

def collisionDetection(adjListMap, newPoints, points,a,b,c):
    bool = 0
    for i in adjListMap:
        for j in adjListMap[i]:
            if c == 0:
                if i == a and j == b or i == b and j == a:
                    continue
            else:
                if b == j or b == i:
                    continue
            intersectX = sys.maxsize
            intersectY = sys.maxsize
            m1 = 0
            b1 = 0
            if newPoints[j][1] == newPoints[i][1]:
                intersectY = newPoints[j][1]
            elif newPoints[j][0] == newPoints[i][0]:
                intersectX = newPoints[j][0]
            else:
                m1 = (newPoints[j][1]-newPoints[i][1])/(newPoints[j][0]-newPoints[i][0])
                b1 = newPoints[i][1] - m1*newPoints[i][0]
            # Test to see if x points or y points are the same
            # This would result in either a 0 slope or divide by 0 error
            if points[1][0] == points[0][0]:
                if intersectX != sys.maxsize:
                    bool = 1
                    break
                elif interxectY != sys.maxsize:
                    intersectX = points[1][0]
                else:
                    intersectX = points[1][0]
                    intersectY = m1*intersectX + b1
            elif points[1][1] == points[0][1]:
                if intersectY != sys.maxsize:
                    bool = 1
                    break
                elif intersectX != sys.maxsize:
                    intersectY = points[1][1]
                else:
                    intersectY = points[1][1]
                    intersectX = (intersectY-b1)/m1
            for k in adjListMap:
                for l in adjListMap[k]:
                    if ((newPoints[l][0] > intersectX > newPoints[k][0] or newPoints[l][0] < intersectX < newPoints[k][0]) and (newPoints[l][1] > intersectY > newPoints[k][1] or newPoints[l][1] < intersectY < newPoints[k][1])) and ((points[0][0] < intersectX < points[1][0] or points[0][0] > intersectX > points[1][0]) and (points[0][1] < intersectY < points[1][1] or points[0][1] > intersectY > points[1][1])) and not intersectX == points[1][0] and not intersectY == points[1][1]:
                        bool = 1
                        break
                if bool==1:
                    break
            if bool==1:
                break
        if bool==1:
            break
    if bool==1:
        return True #the lines collide. This is not a valid point
    return False    #the lines do not collide.

'''
Perform basic search
'''
def backtrace(parent, start, end):
    path = [end]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path

def basicSearch(tree, start, goal):
    parent = {}
    path = []
    visited = [False] * (len(tree)+1)
    squeue = queue.Queue()

    squeue.put(start)
    visited[start] = True
    found=False
    while squeue:
        s = squeue.get(0)
        path.append(s)
        if(s == goal):
            found=True
            return backtrace(parent, start, goal)

            # Get all adjacent vertices of the
            # dequeued vertex s. If a adjacent
            # has not been visited, then mark it
            # visited and enqueue it

        for l in tree[s]:
            if visited[l] == False:
                parent[l] = s
                squeue.put(l)
                visited[l] = True


    return path

'''
Display the RRT and Path
'''
def displayRRTandPath(points, adjListMap, path, robotStart=None, robotGoal=None, polygons=None):
    # Your code goes here
    # You could start by copying code from the function
    # drawProblem and modify it to do what you need.
    # You should draw the problem when applicable.
    _, ax = setupPlot()
    if(robotStart!=None):
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)
    if(robotGoal!=None):
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)

    if(polygons!=None):
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)

    line = []
    for j in adjListMap:
        for k in adjListMap[j]:
            line.append([points[j],points[k]])

    lc = mc.LineCollection(line, linewidths=2, color='black')
    ax.add_collection(lc)

    pathline=[]
    for j in range(1,len(path)):
        pathline.append([points[path[j-1]],points[path[j]]])
    pl = mc.LineCollection(pathline, linewidths=2, color='orange')
    ax.add_collection(pl)


    plt.show()

    #lines = [[(0, 1), (1, 1)], [(2, 3), (3, 3)], [(1, 2), (1, 3)]]
    #lc = mc.LineCollection(lines, linewidths=2)
    #ax.add_collection(lc)
    #plt.scatter(x,y, color='black', marker='o')
    return

'''
Collision checking
'''
def isCollisionFree(robot, point, obstacles):

    # Your code goes here.
    for i in range(0,len(robot)):
        robot[i] = (robot[i][0]+point[0],robot[i][1]+point[1])
        if robot[i][0] < 0 or robot[i][1] < 0 or robot[i][0] > 10 or robot[i][1] > 10:
            return False
        if withinPoly(obstacles, robot[i]):
            return False

    return True

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

    max = 100
    points = dict()
    tree = dict()
    path = []
    tree[1] = [2]
    tree[2] = [1]
    while len(points) != 2:
        x = random.uniform(0.1,9.9)
        y = random.uniform(0.1,9.9)
        if not (withinPoly(obstacles,[x,y]) and not isCollisionFree(robot,(x,y),obstacles)):
            if len(points) == 0:
                points[1] = (x,y)
            else:
                if not lineWithinPoly(obstacles,[(x,y),points[1]]):
                    points[2] = (x,y)
    # Your code goes here.
    i = 2
    while i < max+1:
        x = random.uniform(0.1,9.9)
        y = random.uniform(0.1,9.9)
        if (x,y) in points.values():
            continue
        if withinPoly(obstacles,[x,y]) and not isCollisionFree(robot,(x,y),obstacles):
            continue
        d = sys.maxsize
        index = 0
        for j in list(tree):
            for k in tree[j]:
                if i == 3:
                    if not lineWithinPoly(obstacles,[(x,y),points[k]]):
                        if d > math.sqrt((x-points[k][0])**2+(y-points[k][1])**2):
                            d = math.sqrt((x-points[k][0])**2+(y-points[k][1])**2)
                            index = k
                elif not intersectLines([points[k],(x,y)],tree,points) and not lineWithinPoly(obstacles,[(x,y),points[k]]): #collisionDetection(tree,points,[points[k],(x,y)],j,k,1)
                    if d > math.sqrt((x-points[k][0])**2+(y-points[k][1])**2):
                        d = math.sqrt((x-points[k][0])**2+(y-points[k][1])**2)
                        index = k
        if index != 0:
            points[i] = (x,y)
            if points.get(i) == None:
                points[i] = (x,y)
            if tree.get(index) == None:
                tree[index] = [i]
            else:
                tree[index].append(i)
            tree[i] = [index]
        else:
            continue
        i+=1

    dStart = sys.maxsize
    dGoal = sys.maxsize
    indexS = 0
    indexG = 0
    for i in range(1,len(points)+1):
        if not intersectLines([points[i],startPoint],tree,points) and not lineWithinPoly(obstacles,[startPoint,points[i]]): #collisionDetection(tree,points,[startPoint,points.get(i)],0,i,1)
            if dStart > math.sqrt((startPoint[0]-points[i][0])**2+(startPoint[1]-points[i][1])**2):
                dStart = math.sqrt((startPoint[0]-points[i][0])**2+(startPoint[1]-points[i][1])**2)
                indexS = i
        if not intersectLines([points[i],goalPoint],tree,points) and not lineWithinPoly(obstacles,[goalPoint,points[i]]): #collisionDetection(tree,points,[goalPoint,points.get(i)],0,i,1)
            if dGoal > math.sqrt((goalPoint[0]-points[i][0])**2+(goalPoint[1]-points[i][1])**2):
                dGoal = math.sqrt((goalPoint[0]-points[i][0])**2+(goalPoint[1]-points[i][1])**2)
                indexG = i
    points[max+1] = startPoint
    points[max+2] = goalPoint

    if indexS != 0:
        tree[max+1] = [indexS]
        tree[indexS].append(max+1)
    else:
        print("failed connecting startPoint")

    if indexG != 0:
        tree[max+2] = [indexG]
        tree[indexG].append(max+2)
    else:
        print("failed connecting endPoint")

    path = basicSearch(tree, max+1, max+2)
    return points, tree, path

def withinPoly(polygons, point):
    for poly in polygons:
        patch = createPolygonPatch(poly, 'grey')
        if(patch.get_path().contains_point(point)):
            return True
    return False

def lineWithinPoly(polygons, line):
    for poly in polygons:
        patch = createPolygonPatch(poly, 'grey')

        vcount = 0
        verts=[]
        for vertex in poly:
            verts.append(vertex)
            vcount+=1
        verts.append(verts[0])

        codes = [Path.MOVETO]
        for i in range(0,vcount-1):
            codes.append(Path.LINETO)
        codes.append(Path.CLOSEPOLY)

        path = Path(verts, codes)
        line = np.array([line[0][0],line[0][1],line[1][0],line[1][1]]).reshape(2,2)
        linecodes = [Path.MOVETO,Path.LINETO,Path.CLOSEPOLY]
        line_path = Path(line)

        if(Path.intersects_path(line_path,path)):
            return True
    return False

def main(filename, x1, y1, x2, y2, display=''):
    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print("Robot:")
    print(str(robot))
    print("Pologonal obstacles:")
    for p in range(0, len(obstacles)):
        print(str(obstacles[p]))
    print("")


    # Visualize
    if display == 'display':
        robotStart = [(x + x1, y + y1) for x, y in robot]
        robotGoal = [(x + x2, y + y2) for x, y in robot]
        #drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (6,8)
    #points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (8.3,2.1)
    points[9] = (7.7,6.3)
    points[10] = (9,0.6)
    #points[8] = (9.1, 3.1)
    #points[9] = (8.1, 6.5)
    #points[10] = (0.7, 5.4)
    '''points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)'''

    # Printing the points
    print("")
    print("The input points are:")
    print(str(points))
    print("")

    points, adjListMap = growSimpleRRT(points)
    print("")
    print("The new points are:")
    print(str(points))
    print("")
    print("")
    print("The tree is:")
    print(str(adjListMap))
    print("")

    # Search for a solution
    # change 1 and 20 as you want
    path = basicSearch(adjListMap, 1, 10)
    print("")
    print("The path is:")
    print(str(path))
    print("")

    # Your visualization code
    #if display == 'display':
        #displayRRTandPath(points, adjListMap, path)

    start=datetime.now()
    # Solve a real RRT problem
    points, adjListMap, path = RRT(robot, obstacles, (x1, y1), (x2, y2))
    print("")
    print("runtime = ",(datetime.now()-start).total_seconds(),"seconds")

    print("points:")
    print(str(points))
    print("")
    print("tree:")
    print(str(adjListMap))
    print("")
    print("path:")
    print(str(path))

    # Your visualization code
    if display == 'display':
        displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles)


if __name__ == "__main__":
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()

    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])
    display = ''
    if(len(sys.argv) == 7):
        display = sys.argv[6]

    main(filename, x1, y1, x2, y2, display)
