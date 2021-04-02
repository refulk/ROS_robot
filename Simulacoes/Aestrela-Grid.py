"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import time
import matplotlib.pyplot as plt
import math
import numpy as np
#importa biblioteca para manipulacao de arquivos
import shutil
import os

show_animation = True
numero = 39
origem  = 0
destino = 3
pathName = "dadosAA/"+str(numero)+"_"+str(origem)+"-"+str(destino) #onde serao salvos os resultados

#1-4C-(8-3)(0-5)(8-5)(0-3)
tempo_total = time.time()
dist_total = 0
traj_x = []
traj_y = []

traj_x_temp = []
traj_y_temp = []

class AStarPlanner:
    #Salva Arquivo com o dado desejado
    def salvarArquivo(self,numero,nome,dado):
        global pathName
        global origem
        global destino
        pathName = "dadosA/"+str(numero)+"_"+str(origem)+"-"+str(destino) #onde serao salvos os resultados
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

        if not os.path.exists("dadosA/"+nome):
            os.makedirs("dadosA/"+nome)
        # open a binary file in write mode
        file = open("dadosA/"+nome+"/"+str(numero)+nome+".txt", "wb")
        # save array to the file
        np.save(file, dado)
        # close the file
        file.close

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(ngoal, closed_set)

        return rx, ry

    def calc_dist(self,x1,y1,x2,y2):
      return ((x1-x2)**2 + (y1-y2)**2)**(0.5)

    def calc_final_path(self, ngoal, closedset):
        global tempo_total
        global dist_total
        global traj_x_temp
        global traj_y_temp
        distance, curvas, retaFlag, diagonalFlag, ind = 0, 0, 0, 0, 0
        # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind
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

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, minp):
        """
        calc grid position
        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        # obstacle map generation
        self.obmap = [[False for i in range(self.ywidth)]
                      for i in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
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

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.close()
        #plt.pause(0.01)
        #plt.show()

        traj_x = traj_x_temp
        traj_y = traj_y_temp
        #Salvar RESULTADOS nos arquivos
        a_star.salvarArquivo(numero,"-tempo_total",tempo_total)
        a_star.salvarArquivo(numero,"-dist_total",dist_total)
        a_star.salvarArquivo(numero,"-traj_x",traj_x)
        a_star.salvarArquivo(numero,"-traj_y",traj_y)
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
    main()