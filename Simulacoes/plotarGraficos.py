
#https://matplotlib.org/
#https://www.alura.com.br/artigos/criando-graficos-no-python-com-a-matplotlib
#https://pt.stackoverflow.com/questions/307534/como-salvar-figura-no-python-com-matplotlib
#https://pt.stackoverflow.com/questions/360907/matplotlib-plot-salva-cortado

#https://matplotlib.org/3.1.1/gallery/ticks_and_spines/tick_xlabel_top.html

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import numpy as np

from matplotlib.path import Path
import matplotlib.patches as patches

import pandas as pd


def exibe1():
	print('abc')

def plotar1(xValues, yValues, titulo, eixoX, eixoY):
	#meses = ['Janeiro', 'Fevereiro', 'Marco', 'Abril', 'Maio', 'Junho']
	#valores = [105235, 107697, 110256, 109236, 108859, 109986]

	plt.rcParams['xtick.bottom'] = False #remove o eixo x de baixo
	plt.rcParams['xtick.labelbottom'] = False
	plt.rcParams['xtick.top'] = False	#exibe eixo x em cima
	plt.rcParams['xtick.labeltop'] = True
	plt.gca().invert_yaxis() # eixo y inicia de cima

	# plt.title('Variacao posicao x')
	# plt.xlabel('Tempo (s)')
	# plt.ylabel('Valor de x')
	plt.title(titulo, y=1.08)
	# plt.xlabel(eixoX)
	plt.ylabel(eixoY)	
	# plt.xlim(0, 640)
	# plt.ylim(50, 380)
	for i in range(len(xValues)):
		if i%2 == 0:
			plt.plot(xValues[i],yValues[i],'g')
		else:
			plt.plot(xValues[i],yValues[i],'r')

	#plt.savefig('teste.png', dpi = 300, bbox_inches='tight')
	# plt.savefig('teste.png', format='png')

	plt.show()

def Mapa_plotar():
	plt.rcParams['xtick.bottom'] = False #remove o eixo x de baixo
	plt.rcParams['xtick.labelbottom'] = False
	plt.rcParams['xtick.top'] = False	#exibe eixo x em cima
	plt.rcParams['xtick.labeltop'] = True

	plt.xlim(0, 640)
	plt.ylim(50, 380)
	plt.gca().invert_yaxis() # eixo y inicia de cima

	# plt.title('Variacao posicao x')
	# plt.xlabel('Tempo (s)')
	# plt.ylabel('Valor de x')
	plt.title('titulo', y=1.08)
	# plt.xlabel(eixoX)
	plt.ylabel('eixoY')	
	
	plt.plot([140, 115, 115, 230,325,420,545,545,523,325,115,325,545], [315, 315, 125, 125,125,125,125,315,315,315,180,180,180],'o')
	plt.show()
# Mapa_plotar()

# targets = ['140,315,4.7','115,315,99','115,125,99','230,125,3.16','325,125,99','420,125,3.16','545,125,99','545,315,99','523,315,1.57','325,315,99',
# '115,180,99','325,180,99','545,180,99'
# ]

def boxplot4(value1, value2, value3, value4):
	# value1 = [10,11,12,13,14,15]
	# value2 = [100,110,120,130,140,150]
	# value3 = [1000,1100,1200,1300,1400,1500]
	# value4 = [1000,1100,1200,1300,1400,1500]
	# value1 = [82,76,24,40,67,62,75,78,71,32,98,89,78,67,72,82,87,66,56,52]
	# value2=[62,5,91,25,36,32,96,95,3,90,95,32,27,55,100,15,71,11,37,21]
	# value3=[23,89,12,78,72,89,25,69,68,86,19,49,15,16,16,75,65,31,25,52]
	# value4=[59,73,70,16,81,61,88,98,10,87,29,72,16,23,72,88,78,99,75,30]
	plt.rcParams['figure.figsize'] = (11,7) #mudar tamanho do grafico
	plt.ylim(0, 250)
	box_plot_data=[value1,value2,value3,value4]
	# plt.boxplot(box_plot_data,patch_artist=True,labels=['course1','course2','course3','course4'])
	plt.boxplot(box_plot_data,labels=['1','4','7','10'])

	plt.xlabel("Quantidade de cargas transportadas")
	# plt.ylabel("Tempo (s)")
	# plt.title("Tempo de execução do algoritmo Dijkstra")

	# plt.savefig('item1e-tempoExecucaoDijkstra.png', transparent = False) #salvar imagem com transparencia
	plt.ylabel("Tempo (s)")
	plt.title("Tempo total de execução do algoritmo Dijkstra")

	plt.savefig('item1a-distanciaTotalCALCULADA(custo)Dijkstra.png', transparent = False) #salvar imagem com transparencia

	plt.show()

def barra_dupla_com_erro():
	produtos = ['A','B','C','D','E']
	quantidade1 = [33.6, 16.8, 10.3, 6.2, 3.8]
	quantidade2 = [30.6, 10.8, 5.3, 16.2, 13.8]
	margem_erro1 = [3, 2, 1, 2, 2]
	margem_erro2 = [2, 1, 2, 1, 3]

	ind = [x for x, _ in enumerate(produtos)]

	plt.bar(ind, quantidade2, width=0.8, label='qtd2', color='red', bottom=quantidade1, yerr=margem_erro2,capsize = 5)
	plt.bar(ind, quantidade1, width=0.8, label='qtd1', color='green', yerr=margem_erro1,capsize = 5)

	# plt.xticks(ind, countries)
	plt.xlabel("Produtos (x)")
	plt.ylabel("Participacao em % (y)")
	plt.legend(loc="upper right")
	plt.title("5 produtos")

	plt.show()
# barra_dupla_com_erro()

def plotarTrajetoria(xV, yV):
	# plt.rcParams['xtick.bottom'] = False #remove o eixo x de baixo
	# plt.rcParams['xtick.labelbottom'] = False
	# plt.rcParams['xtick.top'] = False	#exibe eixo x em cima
	# plt.rcParams['xtick.labeltop'] = True
	plt.rcParams['figure.figsize'] = (15,8) #mudar tamanho do grafico

	# plt.xlim(0, 640)
	# plt.ylim(50, 380)
	plt.xlim(0, 100)
	plt.ylim(0, 60)
	# plt.gca().invert_yaxis() # eixo y inicia de cima

	plt.title('Carga 4')
	plt.ylabel('y (cm)')	
	plt.xlabel('x (cm)')	

	# vet1 = [140, 115, 115, 230,325,420,545,545,523,325,115,325,545]
	# vet2 = [315, 315, 125, 125,125,125,125,315,315,315,180,180,180]
	vet3 = [2,95,34,67]
	vet4 = [12,12,30,30]

	# for i in range(len(vet1)):
	# 	vet1[i] = round(vet1[i]/3,2)
	# 	vet2[i] = round(vet2[i]/3,2)
	# for i in range(len(vet3)):
	# 	vet3[i] = round(vet3[i]/3,2)
	# 	vet4[i] = round(vet4[i]/3,2)
	# for j in range(len(xV)):
	# 	for i in range(len(xV[j])):
	# 		xV[j][i] = round(xV[j][i]/3,2)
	# 		yV[j][i] = round(yV[j][i]/3,2)

	
	# plt.plot([140, 115, 115, 230,325,420,545,545,523,325,115,325,545], [315, 315, 125, 125,125,125,125,315,315,315,180,180,180],'o',color='black')
	# plt.plot([230,410,12,628], [250,250,315,310],'o',color='blue')
	# plt.plot(vet1, vet2,'o',color='black')
	plt.plot(vet3, vet4,'o',color='blue')

	for i in range(len(xV)):
		# if i%2 == 0:
		# 	plt.plot(xV[i],yV[i],'g')
		# else:
		# 	plt.plot(xV[i],yV[i],'r')			
		if i== 6:
			plt.plot(xV[i],yV[i],'g')
			# plt.scatter(xV[i],yV[i], color='pink')
		elif i==7:
			plt.plot(xV[i],yV[i],'r')
			# plt.scatter(xV[i],yV[i], color='pink')


	# plt.plot(x[0],y[0],'g',x[1],y[1],'g')
	# plt.grid()


	plt.savefig('item1d-trajetoria_Carga4_Dijkstra.png', transparent = False) #salvar imagem com transparencia

	plt.show()


def plotarEixo(x):

	# xx = [0,1,2,3]
	# x = [0,1,2,3,4,4,4,3,2,1]
	# y = [0,1,1,1,1,2,3,3,3,3]

	plt.rcParams['figure.figsize'] = (15,8) #mudar tamanho do grafico

	# plt.xlim(0, 100)
	plt.ylim(0, 60)
	# for j in range(len(x)):
	# 	for i in range(len(x[j])):
	# 		x[j][i] = round(x[j][i]/3,2)

	vet = []
	vetTemp = []
	i_old = 0
	for i in range(len(x)):
		vet = []
		print('a')
		if i%2 == 0:
			for j in range(0, len(x[i])):
				vet.append(j+i_old)
			print(vet)
			plt.plot(vet,list(reversed(x[i])),'g')
		else:
			for j in range(0, len(x[i])):
				vet.append(j+i_old)
			print(vet)
			plt.plot(vet,list(reversed(x[i])),'r')
		i_old = len(x[i]) + i_old


	# plt.plot(x[0],y[0],'g',x[1],y[1],'g')
	# plt.grid()

	plt.xlabel("Steps")
	plt.ylabel("Posicao em y (cm)")
	plt.title("Posicao do eixo y no tempo - Dijkstra")

	plt.savefig('itemd-eixo_y-Dijkstra.png', transparent = False) #salvar imagem com transparencia
	plt.show()

def bib():
	#https://paulovasconcellos.com.br/15-comandos-de-matplotlib-que-talvez-voc%C3%AA-n%C3%A3o-conhe%C3%A7a-17cf88a75119
	plt.style.use() #usar estilo para mudar cores

	plt.rcParams['figure.figsize'] = (11,7) #mudar tamanho do grafico

	plt.savefig('nome_da_imagem.png', transparent = True) #salvar imagem com transparencia

	### Outra forma de gerar o grafico#
	fig,ax = plt.subplots()
	ax.plot(x, y)
	plt.show()
	######################

	seu_dataframe.plot(x, y, kind='bar') # selecionar o tipo de grafico




# meses = ['Janeiro', 'Fevereiro', 'Marco', 'Abril', 'Maio', 'Junho']
# valores = [105235, 107697, 110256, 109236, 108859, 109986]


# plt.title('Faturamento no primeiro semestre de 2017')
# plt.xlabel('Meses')
# plt.ylabel('Faturamento em R$')
# plt.ylim(100000, 120000)
# plt.plot(meses, valores)

# plt.savefig('teste.png', dpi = 300, bbox_inches='tight')
# # plt.savefig('teste.png', format='png')
# plt.show()



# x = [[0,1,2,3],[4,4,4,3,2,1]]
# y = [[0,1,1,1],[1,2,3,3,3,3]]

def exemploPlotar():
	x=[]
	y=[]

	# xx = [0,1,2,3]
	xx=[]
	xx.append(0)
	xx.append(1)
	xx.append(2)
	xx.append(3)
	yy = [0,1,1,1]

	x.append(xx)
	y.append(yy)
	x.append([4,4,4,3,2,1])
	y.append([1,2,3,3,3,3])
	print(x)
	# x = [0,1,2,3,4,4,4,3,2,1]
	# y = [0,1,1,1,1,2,3,3,3,3]

	for i in range(len(x)):
		if i%2 == 0:
			plt.plot(x[i],y[i],'g')
		else:
			plt.plot(x[i],y[i],'r')


	# plt.plot(x[0],y[0],'g',x[1],y[1],'g')
	# plt.grid()

	plt.show()
