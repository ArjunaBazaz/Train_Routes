import sys
from math import pi, acos, sin, cos
import tkinter as tk

dictionary = {}
edges = []
nodeCities = {}
nodes = {}
lineDict = {}
import heapq

def calcd(node1, node2):
    y1 = float(node1[0])
    x1 = float(node1[1])
    y2 = float(node2[0])
    x2 = float(node2[1])
    # all assumed to be in decimal degrees

    R = 3958.76  # miles = 6371 km
    y1 = y1 * pi / 180.0
    x1 *= pi / 180.0
    y2 *= pi / 180.0
    x2 *= pi / 180.0

    # approximate great circle distance with law of cosines
    return acos(sin(y1) * sin(y2) + cos(y1) * cos(y2) * cos(x2 - x1)) * R


def xyConvert(lat, long):
    x0 = (float(long) + 150) * 10
    y0 = (float(lat) - 71) * (-12)
    return (x0, y0)


with open("rrEdges.txt") as f:
    for line in f:
        x = line.strip().split()
        edges.append(x)
        dictionary[x[0]] = []
        dictionary[x[1]] = []

with open("rrNodes.txt") as f:
    for line in f:
        x = line.strip().split()
        nodes[x[0]] = (x[1], x[2])

with open("rrNodeCity.txt") as f:
    for line in f:
        x = line.strip().split()
        num = x[0]
        name = x[1]
        for y in range(2, len(x)):
            name = name + " " + str(x[y])
        nodeCities[name] = num

for x in edges:
    distance = calcd(nodes[x[0]], nodes[x[1]])
    dictionary[x[0]].append((x[1], distance))
    dictionary[x[1]].append((x[0], distance))


def circleDistance(s, d):
    return calcd(nodes[s], nodes[d])


edges2 = []


def dijkstra(s, d, c1, r):
    tempDict = {}
    step = 0
    closed = set()
    closed.add(s)
    x = (0, s)
    fringe = [x]
    tempDict[s] = ""
    while (len(fringe) > 0):
        if (step > 400):
            step = 0
            r.update()
        v = heapq.heappop(fringe)
        if (v[1] == d):
            return (v[0], tempDict)
        for c in dictionary[v[1]]:
            if (c[0] not in closed):
                tempDict[c[0]] = v[1]
                c1.itemconfig(lineDict[(v[1], c[0])], fill="green")
                edges2.append((v[1], c[0]))
                step = step + 1
                distance = v[0] + c[1]
                y = (distance, c[0])
                heapq.heappush(fringe, y)
                closed.add(c[0])
    print("fail")
    return 0


def A_star(s, d, c1, r):
    tempDict = {}
    step = 0
    closed = set()
    closed.add(s)
    x = (circleDistance(s, d), s, 0)
    fringe = [x]
    tempDict[s] = ""
    while (len(fringe) > 0):
        if (step > 400):
            step = 0
            r.update()
        v = heapq.heappop(fringe)
        if (v[1] == d):
            return (v[2], tempDict)
        for c in dictionary[v[1]]:
            if (c[0] not in closed):
                tempDict[c[0]] = v[1]
                c1.itemconfig(lineDict[(v[1], c[0])], fill="blue")
                step = step + 1
                distance = v[2] + c[1]
                y = (circleDistance(c[0], d) + distance, c[0], distance)
                heapq.heappush(fringe, y)
                closed.add(c[0])
    print("fail")
    return 0


lines = []


def drawLines(c, r):
    for x in edges:
        start = nodes[x[0]]
        end = nodes[x[1]]
        line = c.create_line([xyConvert(start[0], start[1]), xyConvert(end[0], end[1])], tag='map')
        lines.append(line)
        lineDict[tuple(x)] = line
        lineDict[(x[1], x[0])] = line
        c.itemconfig(line, fill="black")  # changes color of one line to black
    # time.sleep(0.1)
    r.update()

def drawLinesFast(c, r):
    for x in edges2:
        c.itemconfig(lineDict[x], fill="black")  # changes color of one line to black
    r.update()


root = tk.Tk()  # creates the frame
canvas = tk.Canvas(root, height=1200, width=1000,
                   bg='white')  # creates a canvas widget, which can be used for drawing lines and shapes
canvas.pack(expand=True)  # packing widgets places them on the board
drawLines(canvas, root)

departure = sys.argv[1]
destination = sys.argv[2]
D = dijkstra(nodeCities[departure], nodeCities[destination], canvas, root)
tempD = D[1]
node = nodeCities[destination]
while tempD[node] != "":
    canvas.itemconfig(lineDict[(node, tempD[node])], fill="red")
    root.update()
    node = tempD[node]
root2 = tk.Tk()
canvas2 = tk.Canvas(root2, height=1200, width=1000,
                    bg='white')  # creates a canvas widget, which can be used for drawing lines and shapes
canvas2.pack(expand=True)  # packing widgets places them on the board
drawLines(canvas2, root2)
A = A_star(nodeCities[departure], nodeCities[destination], canvas2, root2)
tempD = A[1]
node = nodeCities[destination]
while tempD[node] != "":
    canvas2.itemconfig(lineDict[(node, tempD[node])], fill="red")
    root2.update()
    node = tempD[node]

root.mainloop()
