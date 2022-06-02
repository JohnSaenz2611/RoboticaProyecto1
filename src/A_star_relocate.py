"""
States:
    Open set: nodes that still needs to be evaluated
    closed set: all the nodes that have finished been evaluated
"""
from file_reader import File_reader
from math import hypot
import os

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


class A_star_relocate(object):
    def __init__(self, numero_escena) -> None:
        self.numero_escena = numero_escena
        self.path = []
        # Create the 2D array
        spots = [[Spot(i, j) for j in range(rows)] for i in range(cols)]
        for i in range(len(spots)):
            for j in range(len(spots[i])):
                spots[i][j].addNeighbors(spots)

        scene = File_reader(PATH + f'/scenes/Escena-Problema{self.numero_escena}.txt')
        posFinal_x = int((scene.qLoc_x - 0.25) * 2)
        posFinal_y = int(rows - 1 - (scene.qLoc_y - 0.25) * 2)

        posInicial_x = int((scene.qf_x - 0.25) * 2)
        posInicial_y = int(rows - 1 - (scene.qf_y - 0.25) * 2)

        # Definir Obstáculos
        obstacle_x = []
        obstacle_y = []

        for pair in scene.obstacle_list:
            obstacle_x.append(pair[0])
            obstacle_y.append(pair[1])

        obstaclesToPrint = []
        for i in range(len(obstacle_x)):
            obstaclesToPrint.append((obstacle_x[i], obstacle_y[i]))
            
        print(f"Obstaculos: {obstaclesToPrint}")

        for x, y in zip(obstacle_x, obstacle_y):
            spots[int(x)][int(y)].obstacle = True
            obstacles.append(spots[int(x)][int(y)])

        OpenSet = []
        closedSet = []

        start = spots[posInicial_x][posInicial_y] # x, y
        end = spots[posFinal_x][posFinal_y] #

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
                    path_file = open(PATH + f'/paths/Path_list_{self.numero_escena}_ReLoc.txt', 'w')
                    path = self.prepareList(path)
                    for spot in path:
                            self.path.append([spot.x, rows - 1 - spot.y])
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
                        neighbor.h = self.heuristic(neighbor, end)
                        neighbor.f = neighbor.g + neighbor.h
                    
            else:
                # No solution
                print('No Solution')
                gaming = False
                pass


    def prepareList(self, list):
        list.pop()
        reversedList = []
        for i in range(len(list)):
            reversedList.append(list.pop())
        return reversedList

    def heuristic(self, a, b):
        return hypot(a.x - b.x, a.y - b.y)

if __name__ == '__main__':
    a_star_reloc = A_star_relocate(7)