from cProfile import label
import sys
import numpy as np
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt

def TxtReaderCompareKappa(dir1):
	f1 = open(dir1, "r")
	line1 = f1.readline()
	ref_dis,opt_dis = [], []
	while line1:
		linedate1 = line1.split(",")
		if line1 == "\n":
			break
		ref_dis.append(float(linedate1[0]))
		opt_dis.append(float(linedate1[1]))
		line1 = f1.readline()
	
	plt.plot(ref_dis, "g", label = "ref Path")
	plt.plot(opt_dis, "b", label = "opt Path")
	plt.title("distance (Green:ref, Blue: opt, Red: limit)")
	plt.show()

if __name__=="__main__":
	TxtReaderCompareKappa("./resultData/distance.txt")
