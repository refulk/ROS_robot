
#https://kite.com/python/examples/5522/numpy-save-an-array-to-a-text-file

import numpy as np
import os
import plotarGraficos


# pathName = "dadosD/-tempo_total/" #onde estao salvos os resultados

#(3-5)
#4_Cargas/1-4C-(8-3)(0-5)(8-5)(0-3)

# d: cd Renan\UNIFESP\TCC\CodDev\Simulacoes\shrib

def read_files_item1d_plotaEixos_4cargas():
	pathName0 = "dadosD/-traj_x/"
	pathName1 = "dadosD/-traj_y/"
	# save np.load
	np_load_old = np.load

	# modify the default parameters of np.load
	np.load = lambda *a,**k: np_load_old(*a, allow_pickle=True, **k)
	
	filename0 = "-traj_x.txt"
	filename1 = "-traj_y.txt"
	file0 = open(pathName0+str(0)+filename0, "rb")
	file1 = open(pathName1+str(0)+filename1, "rb")

	i = 0

	traj_x = []		#array = [[C1_x,x,x,x],[D1_x,x,x,x],[C2_x,x,x,x],[D2_x,x,x,x]]
	traj_y = []

	soma = 0
	cont = 0
	flag = 0
	
	for ind in range(0,40):
		file0 = open(pathName0+str(ind)+filename0, "rb")
		file1 = open(pathName1+str(ind)+filename1, "rb")

		if(cont == 0):
			traj_x.append(np.load(file0))			#array = [[C1_x,x,x,x],[D1_x,x,x,x],[C2_x,x,x,x],[D2_x,x,x,x]]
			traj_y.append(np.load(file1))	

		# if(ind < 40):
		# 	valores4[cont] += np.round(tempo_total,2)

		# print(np.round(tempo_total,2))
		# valores10[cont] += np.round(tempo_total,2)
		if(flag == 1):
			flag = 0
			cont += 1
			cont = cont%5
		else:
			flag = 1
		
	plotarGraficos.plotarEixo(traj_y)	
	# restore np.load for future normal usage
	np.load = np_load_old
# read_files_item1d_plotaEixos_4cargas()













def read_files_trajetoria():
	pathName0 = "dadosD/-traj_x/"
	pathName1 = "dadosD/-traj_y/"
	# save np.load
	np_load_old = np.load

	# modify the default parameters of np.load
	np.load = lambda *a,**k: np_load_old(*a, allow_pickle=True, **k)
	
	filename0 = "-traj_x.txt"
	filename1 = "-traj_y.txt"
	file0 = open(pathName0+str(0)+filename0, "rb")
	file1 = open(pathName1+str(0)+filename1, "rb")

	i = 0

	traj_x = []		#array = [[C1_x,x,x,x],[D1_x,x,x,x],[C2_x,x,x,x],[D2_x,x,x,x]]
	traj_y = []

	soma = 0
	cont = 0
	flag = 0
	
	for ind in range(0,40):
		file0 = open(pathName0+str(ind)+filename0, "rb")
		file1 = open(pathName1+str(ind)+filename1, "rb")

		if(cont == 0):
			traj_x.append(np.load(file0))			#array = [[C1_x,x,x,x],[D1_x,x,x,x],[C2_x,x,x,x],[D2_x,x,x,x]]
			traj_y.append(np.load(file1))	

		# if(ind < 40):
		# 	valores4[cont] += np.round(tempo_total,2)

		# print(np.round(tempo_total,2))
		# valores10[cont] += np.round(tempo_total,2)
		if(flag == 1):
			flag = 0
			cont += 1
			cont = cont%5
		else:
			flag = 1



	#EXPURGA DADOS INCOERENTES
	# for k in range(len(traj_x)):
	# 	for j in range(len(traj_x[k])-1):
	# 		a = ((traj_x[k][j]-traj_x[k][j+1])**2 + (traj_y[k][j]-traj_y[k][j+1])**2)**(0.5)
	# 		if(a > 100):
	# 			traj_x[k][j] = traj_x[k][j-1]
	# 			traj_y[k][j] = traj_y[k][j-1]
		
	plotarGraficos.plotarTrajetoria(traj_x,traj_y)	
	# restore np.load for future normal usage
	np.load = np_load_old
# read_files_trajetoria()


def read_files_DistanciaTotal():
	pathName = "dadosD/-dist_total/"
	valores1 = [0,0,0,0,0]
	valores4 = [0,0,0,0,0]
	valores7 = [0,0,0,0,0]
	valores10 = [0,0,0,0,0]
	filename = "-dist_total.txt"
	#teste
	file0 = open(pathName+"0"+filename, "rb")
	tempo_total = np.load(file0)	#array = [TempoTotalCarga, TempoTotalDescarga]
	### fim teste
	soma = 0
	cont = 0
	flag = 0
	
	for ind in range(0,100):
		file0 = open(pathName+str(ind)+filename, "rb")
		tempo_total = np.load(file0)	#array = [TempoTotalCarga, TempoTotalDescarga]
		if(ind < 10):
			valores1[cont] += np.round(tempo_total,2)
		if(ind < 40):
			valores4[cont] += np.round(tempo_total,2)
		if(ind < 70):
			valores7[cont] += np.round(tempo_total,2)
		print(np.round(tempo_total,2))
		valores10[cont] += np.round(tempo_total,2)

		if(flag == 1):
			flag = 0
			cont += 1
			cont = cont%5
		else:
			flag = 1

	print ("")
	print (valores1)
	print (valores4)
	print (valores7)
	print (valores10)
	plotarGraficos.boxplot4(valores1,valores4,valores7,valores10)
# read_files_DistanciaTotal()


def read_files_TempoTotal():
	pathName = "dadosD/-tempo_total/"
	valores1 = [0,0,0,0,0]
	valores4 = [0,0,0,0,0]
	valores7 = [0,0,0,0,0]
	valores10 = [0,0,0,0,0]
	filename = "-tempo_total.txt"
	#teste
	file0 = open(pathName+"0"+filename, "rb")
	tempo_total = np.load(file0)	#array = [TempoTotalCarga, TempoTotalDescarga]
	### fim teste
	soma = 0
	cont = 0
	flag = 0
	
	for ind in range(0,100):
		file0 = open(pathName+str(ind)+filename, "rb")
		tempo_total = np.load(file0)	#array = [TempoTotalCarga, TempoTotalDescarga]
		if(ind < 10):
			if(cont == 0):
				print("tempo_total")
				print(tempo_total)
			valores1[cont] += np.round(tempo_total,2)
		if(ind < 40):
			valores4[cont] += np.round(tempo_total,2)
		if(ind < 70):
			valores7[cont] += np.round(tempo_total,2)
		print(np.round(tempo_total,2))
		valores10[cont] += np.round(tempo_total,2)

		if(flag == 1):
			flag = 0
			cont += 1
			cont = cont%5
		else:
			flag = 1

	print ("valores1")
	print (valores1)
	print ("valores4")
	print (valores4)
	print (valores7)
	print (valores10)
	plotarGraficos.boxplot4(valores1,valores4,valores7,valores10)
read_files_TempoTotal()