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
import re
import time
import sys

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
	l_x = np.array(l_x)
	l_y = np.array(l_y)
	abs_arr = np.abs(l_x - l_y)

	idxs = np.logical_and(l_x > 0, l_y > 0)
	abs_arr[~idxs] = -1
	return l_y[idxs], abs_arr[idxs].tolist()

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

def addToDiffList(diffList, newList, step, maxdiff):
	limit = int(maxdiff / step)

	newList = np.array(newList)
	l = newList / step
	l = l.clip(max=limit).astype(np.int)

	diffList = np.array(diffList)
	np.add.at(diffList, l, 1)
	return diffList


class MyParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(2)

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
		self.body = np.array(line.split(",")[:self.height * self.width], dtype = np.float)

# put errors according to depth
def classify_near_far(gt, bins, bin_length, err):
	res = [[] for i in range(len(bins))]

	# find bin: [0-X] -> 0, (X-2X] -> 1, ....
	gt2 = np.floor(np.array(gt) / bin_length)

	err = np.array(err)

	idxs = gt2 < len(bins)

	gt2 = gt2[idxs]
	err = err[idxs]

	# apply data to its corresponding bin
	for i in range(len(err)):
		res[int(gt2[i])].append(err[i])

	return np.array(map(lambda x : np.round(x, 6), res))



def process(args, bin_length, max_depth, max_dist, output_log, show_time):

	bins = range(0, max_depth, bin_length)

	# y axis for plotting depth vs errors
	graph_depth = [[] for i in range(len(bins))]

	logfile = ''
	if output_log:
		# Output log
		logfile = open("output.log", "w")
		logfile.write("filename,total,valid dense, valid gt, filtered dense, filtered gt, valid absdiff\n")

	# Calculate absolute differences with 0.1m of step and a maximum of 100m
	MAXDIFF = 100.0
	STEP = 0.1
	diff_list = [0] * int(MAXDIFF / STEP)

	files_count = 0
	# count the total files .dmap that are available
	files_total = len([f for f in os.listdir(args.dmap_dense) if f.endswith(".dmap")])

	for f in os.listdir(args.dmap_dense):
		if f.endswith(".dmap") and os.path.isfile(args.dmap_gt + '/' + f):
			t = time.time()

			if output_log:
				# log filename
				logfile.write(f + ',')

			dmap_dense = DepthMap(args.dmap_dense + '/' + f)
			dmap_gt = DepthMap(args.dmap_gt + '/' + f)

			if output_log:
				# log total pixels
				logfile.write(str(dmap_dense.height * dmap_dense.width) + ',')
				# log valid pixels
				logfile.write(str(countValid(dmap_dense.body)) + ',')
				logfile.write(str(countValid(dmap_gt.body)) + ',')

			if (max_dist):
				dmap_dense.body = filterList(dmap_dense.body, max_dist)
				dmap_gt.body = filterList(dmap_gt.body, max_dist)

			if output_log:
				# log valid pixels after filtering
				logfile.write(str(countValid(dmap_dense.body)) + ',')
				logfile.write(str(countValid(dmap_gt.body)) + ',')

			if show_time:
				print "Time before absoluteDiffList: ", time.time() - t


			dmap_gt, absdiff_list = absoluteDiffList(dmap_dense.body, dmap_gt.body)

			if show_time:
				print "Time before classify_near_far: ", time.time() - t

			actual_graph = classify_near_far(dmap_gt, bins, bin_length, absdiff_list)

			if show_time:
				print "Time after: ", time.time() - t

			if output_log:
				# log absolute difference valid pixels
				logfile.write(str(countValid(absdiff_list)) + ',')

			diff_list = addToDiffList(diff_list, absdiff_list, STEP, MAXDIFF)

			if show_time:
				print "Time before end: ", time.time() - t

			if output_log:
				logfile.write('\n')

			files_count += 1
			print("Processed: " + f + " - " + str(files_count) + "/" + str(files_total))

			# check that it is not empty data
			if len(actual_graph.shape) == 1:
				np.save("graph_depth"+str(files_count)+".npy", [bins, actual_graph])

			limit = int(MAXDIFF / STEP)
			np.save("diff_list.npy", diff_list[:limit])

			if show_time:
				print "Iteration time: ", time.time() - t

		else:
			print "Warning, no match for file :", f



	if output_log:
		logfile.close()

def main():
	t_orig = time.time()


	parser = MyParser()
	parser.add_argument('dmap_dense', help='dmap dir - dense node')
	parser.add_argument('dmap_gt', help='dmap dir - ground truth')
	parser.add_argument('--output_log', help='bool - true if output a log')
	parser.add_argument('--show_time', help='bool - show computation times')

	parser.add_argument('--max_dist', help='truncate depth maps using this maximum distance')
	args = parser.parse_args()

	assert(os.path.isdir(args.dmap_dense))
	assert(os.path.isdir(args.dmap_gt))

	# TODO: take as argument
	bin_length = 1
	max_depth = 20


	true_values = ['True', 'true', 't', '1']

	output_log = False
	if args.output_log in true_values:
		output_log = True

	show_time = False
	if args.show_time in true_values:
		show_time = True

	max_dist = 0
	if (args.max_dist):
		max_dist = int(args.max_dist)


	process(args, bin_length, max_depth, max_dist, output_log, show_time)

	print "Total Time: ", time.time() - t_orig

if __name__ == "__main__":
	main()
