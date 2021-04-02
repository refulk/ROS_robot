"""
Grid based Dijkstra planning
author: Atsushi Sakai(@Atsushi_twi)
"""
#cd Renan\UNIFESP\TCC\CodDev\Simulacoes\shrib
import time
import matplotlib.pyplot as plt
import math
import numpy as np
#importa biblioteca para manipulacao de arquivos
import shutil
import os

show_animation = True
numero = 39
origem  = 3
destino = 0
pathName = "dados/"+str(numero)+"_"+str(origem)+"-"+str(destino) #onde serao salvos os resultados

#1-4C-(8-3)(0-5)(8-5)(0-3)
tempo_total = time.time()
dist_total = 0
traj_x = []
traj_y = []

traj_x_temp = []
traj_y_temp = []
class Dijkstra:

    #Salva Arquivo com o dado desejado
    def salvarArquivo(self,numero,nome,dado):
        global pathName
        global origem
        global destino
        pathName = "dados/"+str(numero)+"_"+str(origem)+"-"+str(destino) #onde serao salvos os resultados
        print("SALVANDO ARQUIVO")
        # Create target directory & all intermediate directories if don't exists
        if not os.path.exists(pathName):
            os.makedirs(pathName)
        else:
            print("PASTA JA EXISTE, dados serao sobrescritos")
            #break #nao sobrescreve os dados
        # open a binary file in write mode
        file = open(pathName+"/"+str(numero)+nome+".txt", "wb")
        # save array to the file
        np.save(file, dado)

        if not os.path.exists("dados/"+nome):
            os.makedirs("dados/"+nome)
        # open a binary file in write mode
        file = open("dados/"+nome+"/"+str(numero)+nome+".txt", "wb")
        # save array to the file
        np.save(file, dado)
        # close the file
        file.close

    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent = parent  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node

        while 1:
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent = current.parent
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)
                n_id = self.calc_index(node)

                if n_id in closed_set:
                    continue

                if not self.verify_node(node):
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # Discover a new node
                else:
                    if open_set[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_dist(self,x1,y1,x2,y2):
      return ((x1-x2)**2 + (y1-y2)**2)**(0.5)
      
    def calc_final_path(self, goal_node, closed_set):
        global tempo_total
        global dist_total
        global traj_x_temp
        global traj_y_temp
        distance, curvas, retaFlag, diagonalFlag, ind = 0, 0, 0, 0, 0
        # generate final course
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent = goal_node.parent
        while parent != -1:
            n = closed_set[parent]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent = n.parent 
            if((rx[ind] == rx[ind+1]) or (ry[ind] == ry[ind+1])):
              retaFlag = 1
              if(diagonalFlag == 1):
                diagonalFlag = 0
                curvas += 1
            else:
              diagonalFlag = 1
              if(retaFlag == 1):
                retaFlag = 0
                curvas += 1
            traj_x_temp.append(rx[ind])
            traj_y_temp.append(ry[ind])
            distance += self.calc_dist(rx[ind], ry[ind], rx[ind+1], ry[ind+1])   
            ind += 1            
            #print('1') 
            #a = input()
            #plt.plot(rx, ry, "-r")
            #plt.pause(0.01)
        distance = round(distance,2)
        tempo_total = time.time() - tempo_total
        print("\nDistancia: "+str(distance))
        dist_total += distance
        print("qtd Curvas: "+str(curvas))
        return rx, ry

    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main2():
    print(__file__ + " start!!")
    global tempo_total
    global dist_total
    global traj_x_temp
    global traj_y_temp
    global traj_x
    global traj_y
    global origem
    global destino
    tempo_total = time.time()

    posx = [15,0,0,33,0,67,0,0,85]
    posy = [12,0,0,40,0,40,0,0,12]
    # start and goal position
    #42
    #sx = 10.0  # [m]
    #sy = 10.0  # [m]
    #gx = 90.0  # [m]
    #gy = 50.0  # [m]
    ##################
    #40
    #sx = 10.0  # [m]
    #sy = 50.0  # [m]
    #gx = 90.0  # [m]
    #gy = 50.0  # [m]
    ##################
    #42
    sx = posx[origem]  # [m]
    sy = posy[origem]  # [m]
    gx = posx[destino]  # [m]
    gy = posy[destino]  # [m]
    ##################
    #teste
    #sx = 10.0  # [m]
    #sy = 50.0  # [m]
    #gx = 50.0  # [m]
    #gy = 15.0  # [m]
    ##################
    grid_size = 2.0  # [m]
    robot_radius = 5.0  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(0, 100): #parede inferior
        ox.append(i)
        oy.append(0.0)
    for i in range(0, 60): #parede direita
        ox.append(100.0)
        oy.append(i)
    for i in range(0, 100): #parede superior
        ox.append(i)
        oy.append(60.0)
    for i in range(0, 61): #parede esquerda
        ox.append(0.0)
        oy.append(i)
    for i in range(0, 61): #parede esquerda
        ox.append(0.0)
        oy.append(i)

        
    # for i in range(0, 640): #parede inferior
    #     ox.append(i)
    #     oy.append(0.0)
    # for i in range(0, 330): #parede direita
    #     ox.append(640.0)
    #     oy.append(i)
    # for i in range(0, 640): #parede superior
    #     ox.append(i)
    #     oy.append(330.0)
    # for i in range(0, 331): #parede esquerda
    #     ox.append(0.0)
    #     oy.append(i)
    # for i in range(0, 331): #parede esquerda
    #     ox.append(0.0)
    #     oy.append(i)

    #inicio obstaculos 

    #inferior esquerdo
    x1 = 2
    x2 = 5
    y1 = 8
    y2 = 15
    #traco inferior horizontal 
    for i in range(x1, x2):
        ox.append(i)
        oy.append(y1)
    #traco superior horizontal 
    for i in range(x1, x2):
        ox.append(i)
        oy.append(y2)
    #traco principal vertical 
    for i in range(y1, y2):
        ox.append(x1)
        oy.append(i)
        
    #inferior direito
    x1 = 95
    x2 = 98
    y1 = 8
    y2 = 15
    #traco inferior horizontal 
    for i in range(x1+1, x2+1):
        ox.append(i)
        oy.append(y1)
    #traco superior horizontal 
    for i in range(x1+1, x2+1):
        ox.append(i)
        oy.append(y2)
    #traco principal vertical 
    for i in range(y1, y2):
        ox.append(x2)
        oy.append(i)
                
    #central esquerdo
    x1 = 30
    x2 = 37
    y1 = 27
    y2 = 30
    #traco principal horizontal 
    for i in range(x1, x2):
        ox.append(i)
        oy.append(y1)
    #traco esquerdo vertical 
    for i in range(y1, y2):
        ox.append(x1)
        oy.append(i)
    #traco direito vertical 
    for i in range(y1, y2):
        ox.append(x2)
        oy.append(i)
                
    #central direito
    x1 = 63
    x2 = 70
    y1 = 27
    y2 = 30
    #traco principal horizontal 
    for i in range(x1, x2):
        ox.append(i)
        oy.append(y1)
    #traco esquerdo vertical 
    for i in range(y1, y2):
        ox.append(x1)
        oy.append(i)
    #traco direito vertical 
    for i in range(y1, y2):
        ox.append(x2)
        oy.append(i)
    
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)
    
    if show_animation:  # pragma: no cover
        print("a")
        plt.plot(rx, ry, "-r")
        #plt.close()
        plt.pause(0.01)
        plt.show()

        print("TEMPO TOTAL")
        print(tempo_total)

        traj_x = traj_x_temp
        traj_y = traj_y_temp
        #Salvar RESULTADOS nos arquivos
        dijkstra.salvarArquivo(numero,"-tempo_total",tempo_total)
        dijkstra.salvarArquivo(numero,"-dist_total",dist_total)
        dijkstra.salvarArquivo(numero,"-traj_x",traj_x)
        dijkstra.salvarArquivo(numero,"-traj_y",traj_y)
    #return tempo_total, dist_total, traj_x, traj_y

def main():    
    global origem
    global destino
    global numero
    
    global tempo_total
    global dist_total
    global traj_x_temp
    global traj_y_temp
    global traj_x
    global traj_y

    rangeInicio = 0
    rangeFim = 100
    #1-4C-(8-3)(0-5)(8-5)(0-3)
    vetOrigem = [8,8,0,8,0,8,0,8,0,8,0,8,0]
    vetDest   = [3,3,5,5,3,3,5,5,3,3,5,5,3]

    origem = 0
    destino = 0
    indice = 0
    for i in range(rangeInicio,rangeFim):
        tempo_total = time.time()
        dist_total = 0
        traj_x = []
        traj_y = []
        traj_x_temp = []
        traj_y_temp = []
        numero = i
        if(i%10 == 0):
            indice+=1
        
        if(i%2==0): #par
            origem = vetDest[indice-1]
            destino = vetOrigem[indice]
        else: #impar
            origem = vetOrigem[indice]
            destino = vetDest[indice]
        main2()



if __name__ == '__main__':
    main2()