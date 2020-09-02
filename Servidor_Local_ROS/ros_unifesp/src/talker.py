#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

#cd catkin_ws/src/ros_unifesp/src/

import rospy
from std_msgs.msg import String
import sys, tty, termios

#import numpy: the data structure that will handle an image
import numpy as np
#import openCV
import cv2

import math
import time

import sys
import select
import tty
import termios

import Dijkstra
#importa biblioteca para manipulacao de arquivos
import shutil
import os


#import eband_local_planner

print("Iniciando...")
#Caso true, serao enviados os comandos do teclado
#Caso false, o controle sera autonomo
comandarTeclado = True
alvo = 1
pausaInicial = 1 #0 ativa a pausa
#omni = 1 #omnidirecional
# 0 = frente e tras
# 1 = lados
# 2 = todos
# 3 = frente
# 4 = tras
pi = math.pi
path = '/var/www/html/robot_web/Dados/'

imgIndex = 0
obstaculo = False
arestas_excluidas = []

class NonBlockingConsole(object):

    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


    def get_data(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return False



#funcao utilizada para controlar o robo pelo teclado
#funcao extra, utilizada para desenvolvimento e testes
def comandarTecladoFunc(ch):
    '''
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    '''
    print("\n\nComando: " + ch + "\n\n")

    valor = 0
    strValor = ""
    if(ch == 'r'):
        valor = input('Posicao do braco (13): ')
    if(ch == 't'):
        valor = input('Posicao da mao (10): ')
    if(valor < 10):
        strValor = "0" + str(valor)
    else:
        strValor = str(valor)        
    #velocidade 60 anda 1 posicao
    switcher={
        'w':"028499", #frente
        #'w':"028499", #frente
        'a':"02840160", #CURVAesquerda
        'd':"02840060", #CURVAdireita
        's':"038499", #tras
        #'a':"048499", #esquerda
        #'d':"058499", #direita
        'e':"008499", #gira Horario
        'q':"018499", #gira Anti Horario
        'z':"068499", #dFD
        'x':"078499", #dTE
        'c':"088499", #dFE
        'v':"098499", #dTD
        'r':"21"+strValor+"99", #controla Braco (0 - 13)
        #'f':"2202", #braco Baixo
        't':"31"+strValor+"99", #controla Cima (0-18)
        #'g':"3203", #mao Baixo
        'y':"419999", #eletroima on
        'h':"429999", #eletroima off
        'p':"999999", #parado
        'b':"999899"  #para programa
    }
    return switcher.get(ch,"999999")

def getAnImage():
    global cam
    video_capture = cam
    #video_capture = cv2.VideoCapture('video/ros.mp4')

    while(True):
        ret, frame = video_capture.read()
        
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #frame = cv2.resize(frame, (0,0), fx=0.5,fy=0.5)
        #cv2.line(frame,(0,0),(511,511),(255,0,0),5)
        cv2.imshow("Frame",frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.imwrite("images/copy/copy1.jpg", frame)
            break

    video_capture.release()
    cv2.destroyAllWindows()

    global comando
    comando = "999899"

def callback(data):
    global obstaculo
    #print exibe resposta answer do arduino
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    #a = 0
    #Se receber mensagem, existe um obstaculo
    obstaculo = True

def dist(x1,y1,x2,y2):
    return ((x1-x2)**2 + (y1-y2)**2)**(0.5)

def d_p(p1,p2,x2=0,y2=0):  
    x1 = int(p1.split(',')[0])
    y1 = int(p1.split(',')[1])
    if(p2 != ""):
        x2 = int(p2.split(',')[0])
        y2 = int(p2.split(',')[1])
    return dist(x1,y1,x2,y2)

def desenharMapa(img, rx, ry, rt, radius, tetaAlvo, dt, dist):
    global CD_x
    global CD_y
    global CD_x2
    global CD_y2
    rt = round(rt,2)
    tetaAlvo = round(tetaAlvo,2)
    if((rt*100)%(10) == 0):
        rt_s = str(rt)+"0"
    else:
        rt_s = str(rt)
    if((tetaAlvo*100)%(10) == 0):
        tetaAlvo_s = str(tetaAlvo)+"0"
    else:
        tetaAlvo_s = str(tetaAlvo)
    if(radius == 0):
        radius = 0.01
    # Valores aceitaveis: radius entre 8 e 20 / dist entre 2.5 e 4
    message = "X:" + str(rx) + " Y: " + str(ry) + " T: " + rt_s + " = " + tetaAlvo_s
    #print message #exibe posicao
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, message,(5, 450), font, 1,(255,255,255),2) #imagem, texto, localizacao, fonte, espessura, cor,


    ######################################################################
    ######################################################################
    ##### Contorna robo
    #40x30cm (65x48 pontos na imagem)
    #radius = 4.25cm
    #medidas equivalentes a metade do comprimento e metade da largura
    comprimento = (4.7*radius)  #margem de 8 = 5cm
    largura = (3.53 * radius)  #margem de 5 = 3cm
    x1 = int(rx + (math.sin(dt) * comprimento) - (math.cos(dt) * largura))
    y1 = int(ry - (math.cos(dt) * comprimento) - (math.sin(dt) * largura))
    x2 = int(rx + (math.sin(dt) * comprimento) + (math.cos(dt) * largura))
    y2 = int(ry - (math.cos(dt) * comprimento) + (math.sin(dt) * largura))
    x3 = int(rx - (math.sin(dt) * comprimento) + (math.cos(dt) * largura))
    y3 = int(ry + (math.cos(dt) * comprimento) + (math.sin(dt) * largura))
    x4 = int(rx - (math.sin(dt) * comprimento) - (math.cos(dt) * largura))
    y4 = int(ry + (math.cos(dt) * comprimento) - (math.sin(dt) * largura))
    CD_x = int(rx + (math.sin(dt) * 30))
    CD_y = int(ry - (math.cos(dt) * 30))
    CD_x2 = int(rx - (math.sin(dt) * 30))
    CD_y2 = int(ry + (math.cos(dt) * 30))
    pts = np.array([[x1,y1],[x2,y2],[x3,y3],[x4,y4]], np.int32)
    #print(x1-x3,y1-y3)
    cv2.polylines(img,[pts],True,(0,255,255), 1)
    ######################################################################    
    ######################################################################
    #Limites do mapa
    pts = np.array([[0,50],[639,50],[639,380],[0,380]], np.int32)
    cv2.polylines(img,[pts],True,(0,0,0), 2)

    #Carga e desgarca
    #center = (550, 77) #0
    #cv2.circle(img, center, int(round(25)), (0, 0, 255), 2)
    #center = (32, 129) #6
    #cv2.circle(img, center, int(round(25)), (0, 0, 255), 2)
    #center = (33, 220) #8
    #cv2.circle(img, center, int(round(25)), (0, 0, 255), 2)
    #center = (31, 312) #10
    #cv2.circle(img, center, int(round(25)), (0, 0, 255), 2)

    #obstaculos do mapa
    #centro esquerdo
    cv2.line(img,(205,240),(205,260),(255,191,0),2) 
    cv2.line(img,(205,260),(255,260),(255,191,0),2) 
    cv2.line(img,(255,260),(255,240),(255,191,0),2) 
    #centro direito
    cv2.line(img,(385,240),(385,260),(255,191,0),2) 
    cv2.line(img,(385,260),(435,260),(255,191,0),2)
    cv2.line(img,(435,260),(435,240),(255,191,0),2)
    #inferior esquerdo
    cv2.line(img,(22,290),(2,290),(255,191,0),2) 
    cv2.line(img,(2,290),(2,340),(255,191,0),2)
    cv2.line(img,(2,340),(22,340),(255,191,0),2)    
    #inferior direito
    cv2.line(img,(618,285),(638,285),(255,191,0),2) 
    cv2.line(img,(638,285),(638,335),(255,191,0),2)
    cv2.line(img,(638,335),(618,335),(255,191,0),2)    

    #contours = np.array( [ [450,52],[480,52],[480,230],[450,230] ] )
    #cv2.fillPoly(img, pts =[contours], color=(255,191,0))
    #contours = np.array( [ [260,245],[280,245],[280,378],[260,378] ] )
    #cv2.fillPoly(img, pts =[contours], color=(255,191,0))

    
    #cv2.rectangle(img, (0,0), (120,60), (0,255,0), 3) #img, left top, bottom rightm color ,tighness 
    #cv2.rectangle(img, (x1,y1), (x3,y3), (0,255,0), 3) #img, left top, bottom rightm color ,tighness    
    #pts = pts.reshape((-1,1,2))

def getInclination(x_red, y_red, x_blue, y_blue, robot_center_x, robot_center_y, target_x, target_y, omnidirecional):
    default = round ( -1*math.atan2(x_red - x_blue, y_red - y_blue)+pi, 2)
    target = round ( -1*math.atan2(target_x - robot_center_x, target_y - robot_center_y)+pi, 2)
    teorico = default
    direcao = 0 # 0:frente / 1:direita / 2:tras / 3:esquerda
    #################################################################################
    if(omnidirecional == 3): #apenas frente
        return teorico, target, direcao, default
    #################################################################################
    elif(omnidirecional == 4): #apenas tras
        direcao += 2
        teorico += pi
        if(teorico > 2*pi):
            teorico -= 2*pi
        teorico = round(teorico,2)
        return teorico, target, direcao, default
    ################################################################################# 
    elif(omnidirecional == 0): #frente e tras
        delta = abs(target - teorico)
        while(delta > 0.52*pi): #0.5 mais 0.02 de margem
            if(delta > 1.5*pi): #caso em que passa pelo ponto 2pi e 0
                delta = 2*pi - delta
            else:
                direcao += 2
                teorico += pi
                if(teorico > 2*pi):
                    teorico -= 2*pi
                delta = abs(target - teorico)
        teorico = round(teorico,2)
        return teorico, target, direcao, default
    #################################################################################  
    elif(omnidirecional == 1): #lados
        direcao += 1
        teorico += 0.5*pi
        if(teorico > 2*pi):
            teorico -= 2*pi
        delta = abs(target - teorico)
        while(delta > 0.52*pi): #0.5 mais 0.02 de margem
            if(delta > 1.5*pi): #caso em que passa pelo ponto 2pi e 0
                delta = 2*pi - delta
            else:
                direcao += 2
                teorico += pi
                if(teorico > 2*pi):
                    teorico -= 2*pi
                delta = abs(target - teorico)
        teorico = round(teorico,2)
        return teorico, target, direcao, default
    #################################################################################   
    else:#todas direcoes == 2
        delta = abs(target - teorico)
        while(delta > 0.27*pi): #0.25 mais 0.02 de margem
            if(delta > 1.5*pi): #caso em que passa pelo ponto 2pi e 0
                delta = 2*pi - delta
            else:
                direcao += 1
                teorico += 0.5*pi
                if(teorico > 2*pi):
                    teorico -= 2*pi
                delta = abs(target - teorico)
        teorico = round(teorico,2)
        return teorico, target, direcao, default



#Processamento de imagem
def getPosition(target_x=0, target_y=0, tT=99, step=0, omnidirecional=3):
    global x_red 
    global y_red 
    global x_blue  
    global y_blue 
    global robot_center_x 
    global robot_center_y 
    global robot_inclination 
    global img_time
    global imgIndex
    video_capture = cam
    direcao = 0

    ret_val, img = video_capture.read() #obter imagem (frame)
    img_filter = cv2.GaussianBlur(img.copy(), (3, 3), 0) #melhoramento da imagem
    img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV) #altera cores para destacar os pontos de interesse
    img_binary_red = cv2.inRange(img_filter.copy(), THRESHOLD_LOW_RED, THRESHOLD_HIGH_RED) #torna a imagem binaria, apenas os pontos dentro do range ficam brancos
    img_binary_blue = cv2.inRange(img_filter.copy(), THRESHOLD_LOW_BLUE, THRESHOLD_HIGH_BLUE)  #torna a imagem binaria, apenas os pontos dentro do range ficam brancos
    img_binary_red = cv2.dilate(img_binary_red, None, iterations = 1) #dilata a imagem, destacando a parte branca ainda mais
    img_binary_blue = cv2.dilate(img_binary_blue, None, iterations = 1) #dilata a imagem, destacando a parte branca ainda mais
    ########################################################################################
    #Encontra e circula o circulo vermelho
    img_contours = img_binary_red.copy() #apenas realiza a copia
    contours = cv2.findContours(img_contours, cv2.RETR_EXTERNAL, \
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    radius = 0
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x_red, y_red), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] > 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            #center = (int(x_red), int(y_red))
            if radius < MIN_RADIUS:
                center = None
    if center != None:
        cv2.circle(img, center, int(round(radius)), (0, 255, 0)) #desenha circulo em volta do vermelho 
    radius_r = radius
    ########################################################################################
    #Encontrar circulo azul e desenha o circulo de destaque na imagem anterior
    img_contours = img_binary_blue.copy()
    contours = cv2.findContours(img_contours, cv2.RETR_EXTERNAL, \
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    radius = 0
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x_blue, y_blue), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] > 0:
            center = (int(M["m10"] / M["m00"]), 
            int(M["m01"] / M["m00"]))
            if radius < MIN_RADIUS:
                center = None 
    if center != None:
        cv2.circle(img, center, int(round(radius)), (0, 255, 0))
    radius_b = radius
    ########################################################################################
    
    center = (int(x_red), int(y_red))
    cv2.circle(img, center, int(round(3)), (0, 0, 0), 2) #desenha circulo FINAL
    ########################################################################################
    #Exibindo resultados
    '''
    #testar sem os ajustes
    x_red =  x_red * dist_x / CAMERA_WIDTH  
    y_red =  dist_y - (y_red * dist_y / CAMERA_HEIGHT) 
    x_blue = x_blue * dist_x / CAMERA_WIDTH  
    y_blue = dist_y - (y_blue * dist_y / CAMERA_HEIGHT)
    ''' 
    robot_inclination, target_inclination, direcao, default_inclination = getInclination(x_red, y_red, x_blue, y_blue, robot_center_x, robot_center_y, target_x, target_y, omnidirecional)    

    x_red = int(x_red)
    y_red = int(y_red)
    x_blue = int(x_blue)
    y_blue = int(y_blue)
    robot_center_x = int(x_red)
    robot_center_y = int(y_red)
    dist_r_b = dist(x_red,y_red,x_blue,y_blue)
                
    ########################################################################################
    if(step == 2):
        desenharMapa(img, robot_center_x, robot_center_y, robot_inclination, max(radius_r, radius_b),tT, default_inclination, dist_r_b) 
    else:        
        desenharMapa(img, robot_center_x, robot_center_y, robot_inclination, max(radius_r, radius_b),target_inclination, default_inclination, dist_r_b) 
    cv2.line(img,(robot_center_x,robot_center_y),(target_x,target_y),(0,0,255),2) #linha do centro do robo ate o objetivo atual

    if(time.time() - img_time > 0.1):
        img_time = time.time()
        cv2.imwrite("/var/www/html/robot_web/Img/imgVideo"+str(imgIndex)+".png", img)
        if(imgIndex >= 10):
            imgIndex = 0
            if (os.path.isfile("/var/www/html/robot_web/Img/imgVideo"+str(0)+".png")):
                os.rename("/var/www/html/robot_web/Img/imgVideo"+str(0)+".png","/var/www/html/robot_web/Img/imgVideo.png")
        else:
            imgIndex += 1
            if (os.path.isfile("/var/www/html/robot_web/Img/imgVideo"+str(imgIndex)+".png")):
                os.rename("/var/www/html/robot_web/Img/imgVideo"+str(imgIndex)+".png","/var/www/html/robot_web/Img/imgVideo.png")     
    cv2.imshow("Frame",img)
    if cv2.waitKey(1) & 0xFF == ord('l'):
        print('exit')
    return robot_center_x, robot_center_y, robot_inclination, target_inclination, direcao

#Retorna a media dos 10 valores centrais
def getValor(vet):
    np.sort(vet)
    vetLen = len(vet)
    soma = 0
    for i in range((vetLen/2)-5, (vetLen/2)+5):
        soma = soma + vet[i]
    return (soma/10) 

def deltaT(tetaAtual, tetaFinal):
    delta = abs(tetaAtual - tetaFinal)
    if(delta > 4):
        return abs(abs(tetaFinal - tetaAtual)-(2*pi))
    else:
        return delta

#Calcula se deve rotacionar sentido Horario ou AntiHorario
def returnDirection(tetaAtual, tetaFinal):
    global tAtual_Anterior
    global tAtual_Executar
    global tmargem 
    tAtual = tetaAtual
    tFinal = tetaFinal 

    tDelta = tAtual - tFinal
    #quando tDelta = 0 mantem o movimento
    #evita o bug da variacao brusca de 0 a 2pi
    if(abs(tAtual - tAtual_Anterior) > 0.05 or tDelta == 0): 
        tAtual_Anterior = tAtual
        tAtual = tAtual_Executar
        tDelta = tAtual - tFinal
    else:
        tAtual_Executar = tAtual
    if(tDelta < 0):
        if(abs(tDelta) + tmargem < pi):
            #horario
            tmargem = 0
            return 1
        else:
            #antiHorario
            tmargem = -0.05
            return 0
    else:
        if(abs(tDelta) + tmargem < pi):
            #antiHorario
            tmargem = 0.05
            return 0
        else:
            #horario
            tmargem = 0
            return 1


def andarCurva(direcao, velocidade, curva, velCurva):
    if(direcao == 0): #frente
        comandar("02"+velocidade+curva+velCurva)
    elif(direcao == 1): #direita
        comandar("05"+velocidade+curva+velCurva)
    elif(direcao == 2): #tras
        comandar("03"+velocidade+curva+velCurva)
    elif(direcao == 3): #esquerda
        comandar("04"+velocidade+curva+velCurva)

#anda reto
#nao faz curva
def andar(direcao, velocidade):
    if(direcao == 0): #frente
        comandar("02"+velocidade+"99")
    elif(direcao == 1): #direita
        comandar("05"+velocidade+"99")
    elif(direcao == 2): #tras
        comandar("03"+velocidade+"99")
    elif(direcao == 3): #esquerda
        comandar("04"+velocidade+"99")
           
def goTo(tX, tY, tT, tDir):
    global comando
    global pausaInicial
    global obstaculo
    erroXY = 2
    erroTempT = 0.15 #tetaNoCaminho
    erroTempTinicial = 0.02 #tetaInicial
    erroT = 0.04 #tetaFinal
    step = 0
    stop = 0
    stop_time = time.time()
    vetX = [None]*30
    vetY = [None]*30
    vetT = [None]*30
    vetTempT = [None]*30
    erro = 1
    index = 0
    direcao = 0
    omni_ativa = tDir
    for i in range(30):
        vetX[i], vetY[i], vetT[i], vetTempT[i], direcao = getPosition(tX, tY, tT, 0, tDir)
    x = getValor(vetX)
    y = getValor(vetY)
    t = getValor(vetT)
    tempT = getValor(vetTempT)
    #tempT se refere ao Teta durante o deslocamento
    #tT se refere ao Teta desejado na posicao final
    while(1):
        if(obstaculo):
            break
        vetX[index], vetY[index], vetT[index], tempT, direcao = getPosition(tX, tY, tT, step, omni_ativa) 
        omni_ativa = tDir
        x = getValor(vetX)
        y = getValor(vetY)
        t = round(getValor(vetT),2)
        if(pausaInicial==0):
            pausaInicial = input()
            pausaInicial = 1
        index += 1
        if (index == 30):
            index = 0
        if(tT == 99):
            erro = 2
        else:
            erro = 1
        #####################################################
        #primeiro step, apontar para destino com precisao
        if(step == 0):
            if(deltaT(t,tempT) > erroTempTinicial*erro and ((abs(x - tX) > erroXY*(erro*erro) or abs(y - tY) > erroXY*(erro*erro)))):
                if(returnDirection(t, tempT) == 1):
                    if(deltaT(t,tempT) > 0.4):
                        comandar("008499")
                    elif(abs(t-tempT) > 0.15):
                        comandar("006599")    
                    else: 
                        comandar("005599")                       
                else:
                    #gira Anti Horario
                    if(deltaT(t,tempT) > 0.4):
                        comandar("018499")
                    elif(abs(t-tempT) > 0.15):
                        comandar("016599")  
                    else:
                        comandar("015599")                           
            else:
                step = 1
        #####################################################
        #Se nao estiver no objetivo x,y desejado
        if(step == 1):
            if((abs(x - tX) > erroXY*(erro*erro) or abs(y - tY) > erroXY*(erro*erro))):
                #Verifica o teta em movimento
                distancia = dist(x,y,tX,tY)
                #define a velocidade da curva (rotacao)
                if(deltaT(t,tempT) > 0.25):
                    valorCurva = "60"
                else: 
                    valorCurva = "30"
                #define velocidade transacao                    
                if(distancia>50):
                    valorFrente = "84"
                elif(distancia > 30):
                    valorFrente = "75"
                else:
                    valorFrente = "65"
                if(deltaT(t,tempT) > erroTempT): # se o angulo estiver errado
                    erroTempT = 0.04
                    if(returnDirection(t, tempT) == 1):
                        #gira Horario
                        if(distancia>35):
                            andarCurva(direcao,valorFrente,"00",valorCurva)
                        else: 
                            comandar("005599")
                    else:
                        #gira Anti Horario
                        if(distancia>35):
                            andarCurva(direcao,valorFrente,"01",valorCurva)
                        else: 
                            comandar("015599")
                else: #Se o Teta estiver correto             
                    #vai para frente  
                    if(distancia == 30): #corrige a posicao quando estiver proximo do alvo
                        erroTempT = 0.05
                    else:
                        erroTempT = 0.15
                    if(distancia>50):
                        andar(direcao, "84")
                    elif(distancia > 30):
                        andar(direcao, "75")
                    else:
                        andar(direcao, "65")

            else:
                step = 2
        #######################################################
        if(step == 2): #se estiver no objetivo
            omni_ativa = 3 #deve ajustar com o angulo da frente com o objetivo
            #Verifica o teta final
            if(deltaT(t,tT) > erroT and tT != 99):
                stop = 0
                if(returnDirection(t, tT) == 1):
                    #gira Horario
                    if(deltaT(t,tT) > 0.2):
                        comandar("008499")
                    else:
                        comandar("006599")                        
                else:
                    #gira Anti Horario
                    if(deltaT(t,tT) > 0.2):
                        comandar("018499")
                    else:
                        comandar("016599")                        
            else:
                comandar("999999") #parado
                if(stop == 0 and tT != 99):
                    stop = 1
                    stop_time = time.time()
                elif(time.time() - stop_time >= 1):
                    #Chegou no ponto desejado
                    #mantem por 1 segundo para ter certeza
                    comandar("999999") #parado
                    break;

    comandar("999999") #parado
    return x, y, t

#desloca uma distancia. direcao =0 (frente) =2(tras)
def goToDistance(tX, tY, distance, direcao):
    erroXY = 2    
    vetX = [None]*30
    vetY = [None]*30
    index = 0
    for i in range(30):
        vetX[i], vetY[i], vetT, tempT, d = getPosition(tX, tY)
    x = getValor(vetX)
    y = getValor(vetY)
    x1 = x
    y1 = y
    distancia = dist(x,y,x1,y1) 
    while(abs(distance-distancia) > erroXY and distancia < distance):
        vetX[index], vetY[index], vetT, tempT, d = getPosition(tX, tY)
        x = getValor(vetX)
        y = getValor(vetY)
        index += 1
        if (index == 30):
            index = 0
        distancia = dist(x,y,x1,y1) 
        andar(direcao, "65")
    comandar("999999") #parado

#encontra o ponto mais proximo para comecar
def first_point():
    px, py, pt, tempT, direcao = getPosition()
    f_p = 0
    dist_min = d_p(targets[0],"",px,py)
    dist_aux = 0
    for i in range(1,len(targets)):
        dist_aux = d_p(targets[i],"",px,py)
        if(dist_aux<dist_min):
            dist_min = dist_aux
            f_p = i
    return f_p

def carga_descarga(acao):
    #def carga_descarga(acao,ponto_atual=0,dest=0,rota=0,indice=0):  
    #'r':"21"+str(valor), #controla Braco (0 - 13)
    #'f':"2202", #braco Baixo
    #'t':"31"+str(valor), #controla Cima (0-18)
    #'g':"3203", #mao Baixo
    #'y':"4199", #eletroima on
    #'h':"4299", #eletroima off 
    global CD_x
    global CD_y
    global CD_x2
    global CD_y2
    x =  CD_x
    y = CD_y
    carga_time = time.time()
    if(acao == 'D'):
        #descarga
        carga_time = time.time()
        while(time.time() - carga_time < 2):
            comandar("311399") #mao 13
            getPosition()
        carga_time = time.time()
        while(time.time() - carga_time < 2):
            comandar("210199") #braco 1
            getPosition()

        goToDistance(x, y, 14, 0)

        carga_time = time.time()
        while(time.time() - carga_time < 2):
            comandar("429999") #ima OFF
            getPosition()
        x =  CD_x2
        y = CD_y2
        goToDistance(x, y, 14, 2)
        carga_time = time.time()
        while(time.time() - carga_time < 2):
            comandar("211399") #braco 13
            getPosition()
        carga_time = time.time()
        while(time.time() - carga_time < 2):
            comandar("311099") #mao 10    
            getPosition()        
    else:
        #carga 
        carga_time = time.time()
        while(time.time() - carga_time < 2):
            comandar("311399") #mao 13
            getPosition()
        carga_time = time.time()
        while(time.time() - carga_time < 2):
            comandar("210199") #braco 1
            getPosition()
        carga_time = time.time()
        while(time.time() - carga_time < 2):
            comandar("419999") #ima ON  
            getPosition()        
        goToDistance(x, y, 15, 0)
        
        goToDistance(x, y, 15, 2)        

        carga_time = time.time()
        while(time.time() - carga_time < 2):
            comandar("211399") #braco 13
            getPosition()
        carga_time = time.time()
        while(time.time() - carga_time < 2):
            comandar("311099") #mao 10
            getPosition()
        x =  CD_x2
        y = CD_y2

def resetar_grafo():    
    global arestas_excluidas
    for aresta in arestas_excluidas:
        #adiciona aresta em ambos sentidos (True = ida e volta)
        Dijkstra.graph.add_edge(aresta.split(',')[0],aresta.split(',')[1],int(aresta.split(',')[2].split('.')[0]),int(aresta.split(',')[3]), True)
    arestas_excluidas = []

#planeja a rota ate o destino final
def plan_route(dest,acao):
    global obstaculo
    global arestas_excluidas
    ponto_atual = first_point()   
    ponto_aux = 0
    ponto_ant = 0
    dist_aux = 0
    dir_aux = 0
    arestas_excluidas = []
    target_x = int(targets[ponto_atual].split(',')[0])
    target_y = int(targets[ponto_atual].split(',')[1])
    target_t = round(float(targets[ponto_atual].split(',')[2]),2)
    target_d = 2 #primeiro ponto usar omni todas direcoes
    x, y, t = goTo(target_x, target_y, target_t,target_d)
    if(ponto_atual != dest):
        rota = Dijkstra.graph.dijkstra(str(ponto_atual), str(dest))
        indice = 1 #zero eh o ponto atual
        node = False
        
        while(ponto_atual != dest):
            if(obstaculo):
                print("________________________")
                print("Encontrou obstaculo!")
                print(indice)
                obstaculo = False
                #retorna para o ponto anterior (de partida)
                #Se o indice for zero, fica parado
                if(indice > 1):
                    ponto_atual = ponto_atual = ponto_ant
                    #Exclui caminho obstruido
                    dist_aux, dir_aux = Dijkstra.graph.remove_edge(str(ponto_atual), str(ponto_aux))
                    #Armazena caminho a ser excluido
                    arestas_excluidas.append(str(ponto_atual)+','+str(ponto_aux)+','+str(dist_aux)+','+str(dir_aux))               
                    #Calcula nova rota
                    rota = Dijkstra.graph.dijkstra(str(ponto_atual), str(dest))
                    print("Destino antigo (ponto_aux)")
                    print(ponto_aux)
                    print("Ponto atual (Voltar para)")
                    print(ponto_atual)
                    if(len(rota) == 0):
                        #nao existe caminho possivel, aguarda obstaculo sair do caminho
                        ponto_atual = ponto_aux
                        resetar_grafo()
                        indice -= 1
                    else:
                        #se existir caminho zera o indice
                        indice = 0
                    print("Rota entre 5 e 3")
                    print(Dijkstra.graph.dijkstra('5', '3'))
                    print("Rota atual")
                    print(rota)
            else:           
                node = Dijkstra.graph.get_node_pairs_details(str(ponto_atual),rota[indice])
                ponto_ant = ponto_atual
                ponto_atual = int(rota[indice])
            target_x = int(targets[ponto_atual].split(',')[0])
            target_y = int(targets[ponto_atual].split(',')[1])
            if(node != False):
                target_d = int(node[3]) #busca direcao da aresta
            else:
                target_d = 0 #frente e tras
            #cuidado para indice+1 nao estourar o vetor
            if(ponto_atual == dest): #ponto de destino tem o angulo correto
                target_t = round(float(targets[ponto_atual].split(',')[2]),2)
            else: #ponto de passagem nao tem angulo final
                target_t = 99
            x, y, t = goTo(target_x, target_y, target_t, target_d)  
            if(obstaculo): 
                ponto_aux = ponto_atual
                ponto_atual = ponto_ant       
            indice += 1
    resetar_grafo()
    carga_descarga(acao)

#executa a lista de tarefas
def execute_task(Task, historicoFile): 
    line_index = 1
    text = ''
    qtdTask = len(Task)   
    for i in range(qtdTask):        
        plan_route(int(Task[i].split(',')[0]),Task[i].split(',')[1])
        line_index += 1
        if(Task[i].split(',')[1] == 'C'):
            text = 'Carga:'+Task[i].split(',')[0]
        else:
            text = 'Descarga:'+Task[i].split(',')[0]
        gravarArq(historicoFile, ' - ' + text, line_index)
        time.sleep(3)

def comandar(comandoLocal="999999"):    
    global start_time
    global pub
    global tecladoInput
    global comando
    #20 Hz mas a leitura do arduino eh de 10 Hz
    if(time.time() - start_time >= 0.05):
        start_time = time.time()
        # so funciona se o comando por teclado estiver ativado
        if (comandarTeclado):
            comandoLocal = comandarTecladoFunc(tecladoInput)
            comando = comandoLocal
        pub.publish(comandoLocal)
        return 1
    return 0

def gravarArq(historicoFile, content, line_index):
    lines = None
    with open(historicoFile, 'r') as file_handler:
        lines = file_handler.readlines()
    lines.insert(line_index, "'"+content + "',\n")
    with open(historicoFile, 'w') as file_handler:
        file_handler.writelines(lines)

def getTasks():
    print("Em execucao, pressione 's' para sair.")
    with NonBlockingConsole() as nbc:
        tecladoInput = nbc.get_data()
        #Executa enquanto nenhuma tecla eh pressionada
        while True:
            pathTasks = path + 'Tasks/'
            pathExec = path + 'Executados/'
            historicoFile = path + 'historico.js'
            #busca todos os arquivos existentes na pasta
            Task = []
            for archive in os.listdir(pathTasks):
                Task = []
                gravarArq(historicoFile, "<br>> Tarefa "+archive.split('_')[0], 1)
                textFile = open(pathTasks + archive, "r").read()
                textFile = "".join(textFile.splitlines())
                tasks = textFile.split(';') #separa as tarefas
                del(tasks[len(tasks)-1]) # exclui a ultima linha vazia
                for task in tasks:
                    if(task.split(':')[0] == 'Carga'):
                        Task.append(task.split(':')[1]+',C')
                    else:
                        Task.append(task.split(':')[1]+',D')
                execute_task(Task, historicoFile) #executa a tarefa lida de um arquivo

                #movendo o arquivo
                shutil.move(pathTasks+archive, pathExec)  

            tecladoInput = nbc.get_data()
            if (tecladoInput == "s"):
                break #se qlqr tecla for pressionada, para de executar

  

def talker():
    ####################################
    #####       SETUP INICIO       #####
    ####################################
    global start_time
    global comando
    global pub
    global tecladoInput
    global alvo
    tecladoInput = ''
    x, y, t = 0, 0, 0
    
    comando = "9999" #comando que sera enviado para o arduino (pelo talker)
    pub = rospy.Publisher('talker', String, queue_size=0) #declara no talker que ira enviar o comando do notebook para o arduino
    rospy.init_node('talker', anonymous=True) #inicia no talker
    rate = rospy.Rate(1) # 1hz taxa de envio da mensagem, eh utilizado como delay pelo comando rate.sleep()

    ################################################
    rospy.Subscriber('answer', String, callback) #no utilizado para enviar mensagem do arduino para o notebook
    ################################################

    global cam 
    #ler da camera (ou do arquivo de video) 0 ou 1 significa de qual dispositivo de video (camera, conectado no  seu computador), ira buscar
    cam = cv2.VideoCapture(1)
    #################################
    #####       SETUP FIM       #####
    #################################    
    while (alvo != 6 and comando != "999899"): 
        if (comandarTeclado): #apenas se o comando por teclado estiver ativado
            with NonBlockingConsole() as nbc:
                tecladoInput = nbc.get_data()
                #Executa enquanto nenhuma tecla eh pressionada
                while True:
                    getPosition()
                    tecladoInput = nbc.get_data()
                    if (tecladoInput != False):
                        break #se qlqr tecla for pressionada, para de executar
            comandar()                

        else: #autonomo
            getTasks() #ler arquivos e obter tarefas
            alvo = 6
            
    ########################################
    #####       FINALIZAR INICIO       #####
    ########################################
    cam.release() #libera a captura de video
    cv2.destroyAllWindows() #fecha todas as janelas abertas
    #####################################
    #####       FINALIZAR FIM       #####
    #####################################


#Parametros
#(H, S, V) (H = 0,5 * Hgimp) (S = 2,55 * Sgimp) (V = 2,55 * Vgimp)
#BLUE (103, 255, 199)
#RED (162, 87, 176)
#GIMP   (H 0-360) (S 0-100) (V 0-100)
#OPENCV (H 0-179) (S 0-255) (V 0-255)

# Azul
THRESHOLD_LOW_BLUE = (85,155,135);
THRESHOLD_HIGH_BLUE = (125, 255, 195); 

# Vermelho
THRESHOLD_LOW_RED = (160,90,130);
THRESHOLD_HIGH_RED = (190,180,190);

# Raio minimo para o circulo de contorno
MIN_RADIUS = 2
# Distancia visiveis nos eixos
dist_x = 277 
dist_y = 209
# Resolucao desejada
CAMERA_WIDTH = 1024
CAMERA_HEIGHT = 768
# Inicializacao de variaveis globais
x_red = 0 
y_red = 0
x_blue = 0 
y_blue = 0
robot_center_x = 0
robot_center_y = 0
robot_inclination = 0
#sera utilizada para definir a posicao da carga e descarga
CD_x = 0
CD_y = 0
CD_x2 = 0
CD_y2 = 0
#inicial
targets = ['140,315,4.7','115,315,99','115,125,99','230,125,3.16','325,125,99','420,125,3.16','545,125,99','545,315,99','523,315,1.57','325,315,99']

Dijkstra.graph = Dijkstra.Graph([    
    ("0", "1", d_p(targets[0], targets[1]),2),
    ("1", "2", d_p(targets[1], targets[2]),2),
    ("2", "3", d_p(targets[2], targets[3]),2),
    ("3", "4", d_p(targets[3], targets[4]),2),
    ("4", "5", d_p(targets[4], targets[5]),2),
    ("5", "6", d_p(targets[5], targets[6]),2),
    ("6", "7", d_p(targets[6], targets[7]),2),
    ("7", "8", d_p(targets[7], targets[8]),4),
    ("8", "9", d_p(targets[8], targets[9]),4),
    ("4", "9", d_p(targets[9], targets[0]),0),

    ("9", "8", d_p(targets[0], targets[1]),3),
    ("8", "7", d_p(targets[1], targets[2]),3),
    ("7", "6", d_p(targets[2], targets[3]),2),
    ("6", "5", d_p(targets[3], targets[4]),2),
    ("5", "4", d_p(targets[4], targets[5]),2),
    ("4", "3", d_p(targets[5], targets[6]),2),
    ("3", "2", d_p(targets[6], targets[7]),2),
    ("2", "1", d_p(targets[7], targets[8]),2),
    ("1", "0", d_p(targets[8], targets[9]),2),
    ("9", "4", d_p(targets[9], targets[0]),0),
])

target_x = 0
target_y = 0
target_t = round(0,2)

tAtual_Anterior = 0
tAtual_Executar = 0
tmargem = 0

start_time = time.time()
img_time = time.time()

if __name__ == '__main__':
    try:        
        talker()
    except rospy.ROSInterruptException:
        pass
