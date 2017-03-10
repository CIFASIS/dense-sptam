import argparse
import colorsys
import csv
import cv2
import matplotlib.cm as cmx
import matplotlib.colors as colors
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import operator
import os
import pickle

def simplePlot(data):
	plt.plot(data)
	plt.show()

def boxPlot(data):
	plt.boxplot(data, 0, 'bo')
	plt.show()

def histogramPlot(data, bin_factor):
	bins = int(np.amax(data)) * bin_factor
	plt.hist(data, bins)
	plt.show()

def listToArray(data, height, width):
	ret = np.zeros((height, width))
	for i in range(0, height):
		for j in range(0, width):
			ret[i, j] = data[i * width + j]
	return ret

def arrayToList(arr):
	return reduce(operator.add, arr.tolist())

def applyMask(l, mask):
	assert(len(l) == len(mask))
	l_mask = zip(l, mask)
	return map(lambda (x, y): x if y else -1, l_mask)

def filterList(l, limit):
	return map(lambda x: x if x <= limit else -1, l)

def filterMap(dmap, limit):
	for i in range(0, dmap.shape[0]):
		for j in range(0, dmap.shape[1]):
			if (dmap[i, j] > limit):
				dmap[i, j] = -1

def countValid(l):
	return len(filter(lambda x: x >= 0, l))

def intersectMask(dmap_x, dmap_y):
	assert(dmap_x.shape[0] == dmap_y.shape[0])
	assert(dmap_x.shape[1] == dmap_y.shape[1])
	ret = np.zeros(dmap_x.shape[0], dmap_x.shape[1])
	for i in range(0, dmap_x.shape[0]):
		for j in range(0, dmap_x.shape[1]):
			if (dmap_x[i, j] >= 0 and dmap_y[i, j] >= 0):
				ret[i, j] = 1
	return ret

def absoluteDiffList(l_x, l_y):
	assert(len(l_x) == len(l_y))
	return map(lambda (a, b): abs(a - b) if a >= 0 and b >= 0 else -1, zip(l_x, l_y))

def absoluteDiffMap(dmap_x, dmap_y):
	assert(dmap_x.shape[0] == dmap_y.shape[0])
	assert(dmap_x.shape[1] == dmap_y.shape[1])
	ret = np.full((dmap_x.shape[0], dmap_x.shape[1]), -1)
	for i in range(0, dmap_x.shape[0]):
		for j in range(0, dmap_x.shape[1]):
			if (dmap_x[i, j] >= 0 and dmap_y[i, j] >= 0):
				ret[i, j] = abs(dmap_x[i, j] - dmap_y[i, j])
	return ret

def getRGBcolor01(val, high, hsv_s = 0.8, hsv_v = 0.8, factor = 1):
	hsv = ((1 - float(val) / float(high)) * factor, hsv_s, hsv_v)
	return colorsys.hsv_to_rgb(*hsv)

def getRGBcolor(val, high, hsv_s = 0.8, hsv_v = 0.8, factor = 1):
	return tuple(int(i * 255) for i in getRGBcolor01(val, high, hsv_s, hsv_v, factor))

def genRGBcolors(N):
	return map(lambda x: getRGBcolor(x, N), range(N))

def doColorArray(arr, limit = 0):
	background = (127, 127, 127)
	if (not limit):
		limit = np.amax(arr)
	img = np.full((arr.shape[0], arr.shape[1], 3), background, np.uint8)
	for i in range(0, arr.shape[0]):
		for j in range(0, arr.shape[1]):
			if (arr[i, j] >= 0):
				img[i, j] = getRGBcolor(arr[i, j], limit)
	return img

def saveImage(filename, img):
	cv2.imwrite(filename, img)

def plotSaveImage(filename, img):
	plt.imshow(img)
	plt.savefig(filename, dpi=300)

def genSpectrum(N):
	fig = plt.figure()
	ax = fig.add_subplot(111)
	plt.axis('scaled')
	ax.set_xlim([0, N])
	ax.set_ylim([-1, 1])
	for i in range(N):
		col = getRGBcolor01(i, N)
		rect = plt.Rectangle((i, -1), 2, 2, facecolor=col)
		ax.add_artist(rect)
		ax.set_yticks([])
	plt.savefig('spectrum.png')

def addToDiffList(diffList, newList, step, maxdiff):
	limit = int(MAXDIFF / STEP)
	for i in newList:
		if (i < 0):
			continue;
		pos = int(i / step)
		if (pos > limit):
			pos = limit
		diffList[pos] = diffList[pos] + 1
	return diffList

class DepthMap:

	def __init__(self, filename):
		self.filename = filename
		dmapfile = open(self.filename, "r")
		self.parseheader(dmapfile.readline())
		self.parsebody(dmapfile.readline())
		dmapfile.close()

	def parseheader(self, line):
		data = line.split(",")
		self.height = int(data[0])
		self.width = int(data[1])

	def parsebody(self, line):
		self.body = map(float, line.split(",")[:self.height * self.width])

parser = argparse.ArgumentParser()
parser.add_argument('dmap_dense', help='dmap dir - dense node')
parser.add_argument('dmap_gt', help='dmap dir - ground truth')
parser.add_argument('--max_dist', help='truncate depth maps using this maximum distance')
parser.add_argument('--max_colors', help='number of colors to use')
parser.add_argument('--gen_spectrum', help='generate spectrum of N colors')
args = parser.parse_args()

assert(os.path.isdir(args.dmap_dense))
assert(os.path.isdir(args.dmap_gt))

max_dist = 0
if (args.max_dist):
	max_dist = int(args.max_dist)

max_colors = 0
if (args.max_colors):
	max_colors = int(args.max_colors)

if (args.gen_spectrum):
	genSpectrum(int(args.gen_spectrum))
	exit(0)

# Output log
logfile = open("output.log", "w")
logfile.write("filename,total,valid dense, valid gt, filtered dense, filtered gt, valid absdiff\n")

# Calculate absolute differences with 0.1m of step and a maximum of 100m
MAXDIFF = 100.0
STEP = 0.1
diff_list = [0] * int(MAXDIFF / STEP)

files_count = 0
files_total = len(os.listdir(args.dmap_dense))
for f in os.listdir(args.dmap_dense):
	if f.endswith(".dmap") and os.path.isfile(args.dmap_gt + '/' + f):
		# log filename
		logfile.write(f + ',')

		dmap_dense = DepthMap(args.dmap_dense + '/' + f)
		dmap_gt = DepthMap(args.dmap_gt + '/' + f)

		# log total pixels
		logfile.write(str(dmap_dense.height * dmap_dense.width) + ',')
		# log valid pixels
		logfile.write(str(countValid(dmap_dense.body)) + ',')
		logfile.write(str(countValid(dmap_gt.body)) + ',')

		if (max_dist):
			dmap_dense.body = filterList(dmap_dense.body, max_dist)
			dmap_gt.body = filterList(dmap_gt.body, max_dist)

		# log valid pixels after filtering
		logfile.write(str(countValid(dmap_dense.body)) + ',')
		logfile.write(str(countValid(dmap_gt.body)) + ',')

		absdiff_list = absoluteDiffList(dmap_dense.body, dmap_gt.body)

		# log absolute difference valid pixels
		logfile.write(str(countValid(absdiff_list)) + ',')

		#output_file = os.path.splitext(f)[0] + '_absdiff.png'
		#dmap_diff_arr = listToArray(absdiff_list, dmap_dense.height, dmap_dense.width)
		#plotSaveImage(output_file, doColorArray(dmap_diff_arr, max_colors));
		#saveImage(output_file, doColorArray(dmap_diff_arr, max_colors))

		diff_list = addToDiffList(diff_list, absdiff_list, STEP, MAXDIFF)
		logfile.write('\n')

		files_count += 1
		print("Processed: " + f + " - " + str(files_count) + "/" + str(files_total))

diff_list_file = open('diff_list.txt', 'w')
for item in diff_list:
	diff_list_file.write("%s," % item)
diff_list_file.close()

#histogramPlot(reduce(operator.add, diff_list), 100)
logfile.close()
