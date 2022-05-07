"""
States:
    Open set: nodes that still needs to be evaluated
    closed set: all the nodes that have finished been evaluated
"""
from math import hypot
import os

FILE_NUMBER = 6
PATH = os.path.dirname(os.path.abspath(__file__))
obstacles = []

# How many columns and rows?
cols = 8
rows = 10

#Width and height of each cell of grid
path = []
current = []

class Spot(object):
    def __init__(self, x, y):
        self.f = 0
        self.g = 0
        self.h = 0
        self.x = x # posicion x del punto en el espacio
        self.y = y # Posicion y del punto en el espacio
        self.Neighbors = []
        self.previous = None
        self.obstacle = False

    def addNeighbors(self, spots):
        if self.x >= 1:
            self.Neighbors.append(spots[self.x - 1][self.y])
        if self.x < (cols - 1):
            self.Neighbors.append(spots[self.x + 1][self.y])
        if self.y >= 1:
            self.Neighbors.append(spots[self.x][self.y - 1])
        if self.y < (rows - 1):
            self.Neighbors.append(spots[self.x][self.y + 1])

def prepareList(list):
    list.pop()
    reversedList = []
    for i in range(len(list)):
        reversedList.append(list.pop())
    return reversedList

def heuristic(a, b):
    return hypot(a.x - b.x, a.y - b.y)

# Create the 2D array
spots = [[Spot(i, j) for j in range(rows)] for i in range(cols)]
for i in range(len(spots)):
    for j in range(len(spots[i])):
        spots[i][j].addNeighbors(spots)


# Definir Obstáculos
obstacle_x = []
obstacle_y = []
obstacle_file = open(PATH + f'/obstacles/Obstacles_{FILE_NUMBER}.txt')
obstacle_list = obstacle_file.read().split()
#Posicion Final
posFinal = obstacle_list.pop()
#Posicion Inicial
posInicial = obstacle_list.pop()

for pair in obstacle_list:
    obstacle_x.append(int(pair[0]))
    obstacle_y.append(rows - 1 - int(pair[2]))

obstaclesToPrint = []
for i in range(len(obstacle_x)):
    obstaclesToPrint.append((obstacle_x[i], obstacle_y[i]))
    
print(f"Obstaculos: {obstaclesToPrint}")

for x, y in zip(obstacle_x, obstacle_y):
    spots[int(x)][int(y)].obstacle = True
    obstacles.append(spots[int(x)][int(y)])

OpenSet = []
closedSet = []

start = spots[int(posInicial[0])][rows - 1 - int(posInicial[2])] # x, y
end = spots[int(posFinal[0])][rows - 1 - int(posFinal[2])]

OpenSet.append(start)
current = start

gaming = True
while gaming:
    #Find the path
    path = []
    temp = current
    path.append(temp)
    #As long as the temp has a previous
    while temp.previous:
        current = temp
        path.append(temp.previous)
        temp = temp.previous

    # Find the one to evaluate next
    if len(OpenSet) > 0:
        winner = 0

        for i in range(len(OpenSet)):
            if OpenSet[i].f < OpenSet[winner].f:
                winner = i

        current = OpenSet[winner] 
        if current == end:
            #Find the path
            path = []
            temp = current
            path.append(temp)
            #As long as the temp has a previous
            while temp.previous:
                current = temp
                path.append(temp.previous)
                temp = temp.previous
            #system('cls')
            print('Finish!')
            gaming = False
            path_file = open(PATH + f'/paths/Path_list_{FILE_NUMBER}.txt', 'w')
            path = prepareList(path)
            for spot in path:
                    path_file.write(f'{spot.x},{rows - 1 - spot.y}\n')
            path_file.close()
        try:
            OpenSet.remove(current)
        except ValueError as e:
            pass
            #print(e)

        closedSet.append(current)

        # Verify Neighbors of the current cell
        neighbors = current.Neighbors
        for neighbor in neighbors:
            if not(neighbor in closedSet)  and not(neighbor.obstacle): # ceck if neighbor is available to visit
                temp = current.g + 1

                if neighbor in OpenSet:
                    if temp < neighbor.g:
                        neighbor.g = temp
                else:
                    newpath = True
                    neighbor.g = temp
                    OpenSet.append(neighbor)
        
                neighbor.previous = current
            # We aply Heuristics
            if newpath:
                neighbor.h = heuristic(neighbor, end)
                neighbor.f = neighbor.g + neighbor.h
            
    else:
        # No solution
        print('No Solution')
        gaming = False
        pass
