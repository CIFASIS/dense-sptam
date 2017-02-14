import argparse
import colorsys
import csv
import cv2
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import operator
import os

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

def getRGBcolor(val, high, hsv_s = 0.8, hsv_v = 0.8, factor = 0.7):
	hsv = ((1 - float(val) / float(high)) * factor, hsv_s, hsv_v)
	rgb = tuple(int(i * 255) for i in colorsys.hsv_to_rgb(*hsv))
	return rgb

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
args = parser.parse_args()

assert(os.path.isdir(args.dmap_dense))
assert(os.path.isdir(args.dmap_gt))

max_dist = 0
if (args.max_dist):
	max_dist = int(args.max_dist)

max_colors = 0
if (args.max_colors):
	max_colors = args.max_colors

diff_list = []
for f in os.listdir(args.dmap_dense):
	if f.endswith(".dmap"):
		log_entry = [('filename', f)]

		dmap_dense = DepthMap(args.dmap_dense + '/' + f)
		dmap_gt = DepthMap(args.dmap_gt + '/' + f)

		log_entry.append(('total pixels', dmap_dense.height * dmap_dense.width))
		log_entry.append(('valid dense,gt', countValid(dmap_dense.body), countValid(dmap_gt.body)))

		if (max_dist):
			dmap_dense.body = filterList(dmap_dense.body, max_dist)
			dmap_gt.body = filterList(dmap_gt.body, max_dist)

		log_entry.append(('valid filtered', countValid(dmap_dense.body), countValid(dmap_gt.body)))

		absdiff_list = absoluteDiffList(dmap_dense.body, dmap_gt.body)

		log_entry.append(('valid absdiff', countValid(absdiff_list)))

		output_file = os.path.splitext(f)[0] + '_absdiff.png'
		dmap_diff_arr = listToArray(absdiff_list, dmap_dense.height, dmap_dense.width)
		saveImage(output_file, doColorArray(dmap_diff_arr, max_colors))

		print(log_entry)

histogramPlot(diff_list, 100)

