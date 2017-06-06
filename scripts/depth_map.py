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
import depth_map_utilities as dmu

# filter l1 and l2 where 0 <= l1 <= limit and 0 <= l2 <= limit
def filterLists(l1, l2, limit):
	assert(len(l1) == len(l2))

	l1 = np.array(l1)
	l2 = np.array(l2)
	l1[l1 >= limit] = -1
	l2[l2 >= limit] = -1

	idxs = np.logical_and(l1 >= 0, l2 >= 0)

	return l1[idxs], l2[idxs]

# add newList values to diffList
def addToDiffList(diffList, newList, step, maxdiff):
	limit = int(maxdiff / step)

	newList = np.array(newList)
	l = newList / step
	l = l.clip(max=limit).astype(np.int)

	diffList = np.array(diffList)
	np.add.at(diffList, l, 1)
	return diffList


# put errors according to depth
def classify_near_far(gt, err, bins, bin_length):
	res = [[] for i in range(len(bins))]

	# find bin: [0-X] -> 0, (X-2X] -> 1, ....
	gt2 = np.floor(np.array(gt) / bin_length).astype(np.int)

	# apply data to its corresponding bin
	for i in range(len(err)):
		res[gt2[i]].append(err[i])

	return np.array(map(lambda x : np.round(x, 6), res))


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




def process(args, bin_length, max_dist, output_log, show_time):

	bins = range(0, max_dist, bin_length)

	# y axis for plotting depth vs errors
	graph_depth = [[] for i in range(len(bins))]

	logfile = ''
	if output_log:
		# Output log
		logfile = open("output.log", "w")
		logfile.write("filename,total,valid dense, valid gt, filtered dense, filtered gt, valid absdiff\n")

	# Calculate absolute differences with 0.1m of step and a maximum of MAXDIFF

	diff_list = [0] * int(dmu.MAXDIFF_FIRST_GRAPH / dmu.STEP_FIRST_GRAPH)

	files_count = 0
	# count the total files .dmap that are available
	files_total = len([f for f in os.listdir(args.dmap_dense) if f.endswith(".dmap")])

	for f in os.listdir(args.dmap_dense):
		if f.endswith(".dmap") and os.path.isfile(args.dmap_gt + '/' + f):
			t = time.time()

			if output_log:
				# log filename
				logfile.write(f + ',')

			dmap_dense_o = DepthMap(args.dmap_dense + '/' + f)
			dmap_gt_o = DepthMap(args.dmap_gt + '/' + f)

			if output_log:
				# log total pixels
				logfile.write(str(dmap_dense_o.height * dmap_dense_o.width) + ',')
				# log valid pixels
				logfile.write(str(countValid(dmap_dense_o.body)) + ',')
				logfile.write(str(countValid(dmap_gt_o.body)) + ',')

			dmap_dense, dmap_gt = filterLists(dmap_dense_o.body, dmap_gt_o.body, max_dist)

			if output_log:
				# log valid pixels after filtering
				logfile.write(str(countValid(dmap_dense)) + ',')
				logfile.write(str(countValid(dmap_gt)) + ',')

			absdiff_list = np.abs(dmap_dense - dmap_gt)

			if show_time:
				print "Time before classify_near_far: ", time.time() - t

			actual_graph = classify_near_far(dmap_gt, absdiff_list, bins, bin_length)

			if show_time:
				print "Time after: ", time.time() - t

			if output_log:
				# log absolute difference valid pixels
				logfile.write(str(countValid(absdiff_list)) + ',')

			diff_list = addToDiffList(diff_list, absdiff_list.tolist(), dmu.STEP_FIRST_GRAPH, dmu.MAXDIFF_FIRST_GRAPH)

			if show_time:
				print "Time before end: ", time.time() - t

			if output_log:
				logfile.write('\n')

			files_count += 1
			print("Processed: " + f + " - " + str(files_count) + "/" + str(files_total))

			# check that it is not empty data
			if len(actual_graph.shape) == 1:
				np.save("graph_depth"+str(files_count)+".npy", [bins, actual_graph])

			limit = int(dmu.MAXDIFF_FIRST_GRAPH / dmu.STEP_FIRST_GRAPH)
			np.save("diff_list.npy", diff_list[:limit])

			if show_time:
				print "Iteration time: ", time.time() - t

		else:
			print "Warning, no match for file :", f



	if output_log:
		logfile.close()

def main():
	parser = MyParser()
	parser.add_argument('dmap_dense', help='dmap dir - dense node')
	parser.add_argument('dmap_gt', help='dmap dir - ground truth')
	parser.add_argument('--output_log', help='bool - true if output a log')
	parser.add_argument('--show_time', help='bool - show computation times')
	parser.add_argument('--bin_length', help='float - length of bin in graphs')
	parser.add_argument('--max_dist', help='int - truncate depth maps using this maximum distance')

	args = parser.parse_args()

	if not(os.path.isdir(args.dmap_dense)) or not (os.path.isdir(args.dmap_gt)):
		print "Some of the provided dirs does not exist"
		return

	true_values = ['True', 'true', 't', '1']

	output_log = False
	if args.output_log in true_values:
		output_log = True

	show_time = False
	if args.show_time in true_values:
		show_time = True

	max_dist = 20
	if (args.max_dist):
		max_dist = int(args.max_dist)

	bin_length = 1
	if (args.bin_length):
		bin_length = float(args.bin_length)


	t_orig = time.time()

	process(args, bin_length, max_dist, output_log, show_time)

	print "Total Time: ", time.time() - t_orig

if __name__ == "__main__":
	main()
