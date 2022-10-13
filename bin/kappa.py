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
	min_limit = []
	max_limit = []
	for i in range(0, len(test_k)):
		min_limit.append(-0.1)
		max_limit.append(0.1)

	plt.plot(test_k, "g")
	plt.plot(opt_k, "b")
	plt.plot(min_limit, "r")
	plt.plot(max_limit, "r")
	#plt.plot(v_limit, "r")
	y_tick = np.arange(-0.1,0.1, 0.02)
	plt.yscale('linear')
	plt.ylim((-0.15, 0.15))
	plt.yticks(y_tick)
	plt.title("kappa (Green:ref, Blue: opt, Red: limit)")
	plt.show()

if __name__=="__main__":
	TxtReaderCompareKappa("./resultData/referPath.txt", "./resultData/optTraj.txt")
