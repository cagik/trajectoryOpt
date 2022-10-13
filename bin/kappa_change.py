from cProfile import label
import sys
import numpy as np
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt

def TxtReaderCompareKappa(dir1, dir2):
	f1 = open(dir1, "r")
	line1 = f1.readline()
	test_k = []
	while line1:
		linedate1 = line1.split(",")
		if line1 == "\n":
			break
		test_k.append(float(linedate1[3]))
		line1 = f1.readline()
	
	f2 = open(dir2, "r")
	line2 = f2.readline()
	opt_k = []
	while line2:
		linedate2 = line2.split(",")
		if line2 == "\n":
			break
		opt_k.append(float(linedate2[4]))
		line2 = f2.readline()
	
	#plt.axis("equal")
	test_kp = []
	opt_kp = []
	for i in range(1, len(test_k) - 2):
		test_kp.append((test_k[i] - test_k[i-1])/1)
		#print(test_k[i] - test_k[i-1]/0.4)
	for i in range(1, len(opt_k) - 2):
		opt_kp.append((opt_k[i] - opt_k[i-1])/1)
	plt.plot(test_kp, "g", label = "ref Path")
	plt.plot(opt_kp, "b", label = "opt Path")
	#plt.plot(v_limit, "r")
	y_tick = np.arange(-0.035,0.035, 0.005)
	plt.yscale('linear')
	#plt.ylim((-0.2, 0.2))
	plt.yticks(y_tick)
	plt.legend()
	plt.title("rate of kappa (Green:ref, Blue: opt, Red: limit)")
	plt.show()

if __name__=="__main__":
	TxtReaderCompareKappa("./resultData/referPath.txt", "./resultData/optTraj.txt")
