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

# filter l1 and l2 where 0 <= l1 <= limit and min_dist <= l2 <= limit
def filterLists(l1, l2, min_dist, limit):
  assert(len(l1) == len(l2))

  idxs = np.logical_and(l1 >= 0, l2 >= min_dist)

  # limit > 0 means the parameter max_dist is specified by the user
  if limit > 0:
    idxs = np.logical_and(idxs , l2 < limit)

  return l1[idxs], l2[idxs]

# add newList values to diffList
def addToDiffList(diffList, newList, step, maxdiff):
  limit = int(maxdiff / step)

  l = newList / step
  l = l.clip(max=limit).astype(np.int)

  diffList = np.array(diffList)
  np.add.at(diffList, l, 1)
  return diffList


# put errors according to depth
def classify_near_far(gt, err, bins, bin_length, min_dist):
  res = [[] for i in range(len(bins))]

  # find bin: [0-X] -> 0, (X-2X] -> 1, ....
  gt2 = np.floor(np.array(gt) / bin_length).astype(np.int) - np.int(min_dist)

  # apply data to its corresponding bin
  for i in range(len(err)):
    if gt2[i] > len(res) - 1:
      diff = gt2[i] - (len(res) - 1)
      # should fill res until len(res) - 1 == gt2[i]
      for j in range(diff):
        res.append([])

    res[gt2[i]].append(err[i])

  bins = np.arange(0, bin_length * len(res), bin_length) + min_dist
  return np.array(map(lambda x : np.round(x, 6), res)), bins


# class for showing info on bad usage
class MyParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(2)

# class for reading dmap files
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

  # check if there are negative values other than -1
  def check_sanity(self):
    body = self.body
    body = body[body < 0]
    body = body[body != -1]
    if len(body) > 0:
      print "WARNING: there are negative values in the dmap!! (different from -1, " + self.filename + ")"


  def parsebody(self, line):
    self.body = np.array(line.split(",")[:self.height * self.width], dtype = np.float)
    self.check_sanity()



# main process
def process(args, bin_length, min_dist, max_dist, output_log, show_time):

  # x axis of graphs
  # y axis for plotting depth vs errors
  bins = []
  max_bins = []

  if max_dist > 0:
    bins = np.arange(min_dist, max_dist, bin_length)

  logfile = ''
  if output_log:
    # Output log
    logfile = open("output.log", "w")
    logfile.write("filename,total,valid dense, valid gt, filtered dense, filtered gt, valid absdiff\n")

  # Calculate absolute differences with dmu.STEP_FIRST_GRAPH (m) of step and a maximum of dmu.MAXDIFF_FIRST_GRAPH (m)
  limit = int(dmu.MAXDIFF_FIRST_GRAPH / dmu.STEP_FIRST_GRAPH)
  diff_list = [0] * limit

  files_count = 0
  # count the total files .dmap that are available
  files_total = len([f for f in os.listdir(args.dmap_dense) if f.endswith(".dmap")])

  # loop through files
  for f in os.listdir(args.dmap_dense):
    # see if file is a depth map
    if f.endswith(".dmap") and os.path.isfile(args.dmap_gt + '/' + f):
      t = time.time()

      if output_log:
        # log filename
        logfile.write(f + ',')

      # convert file to DepthMap class
      dmap_dense_o = DepthMap(args.dmap_dense + '/' + f)
      dmap_gt_o = DepthMap(args.dmap_gt + '/' + f)

      if output_log:
        # log total pixels
        logfile.write(str(dmap_dense_o.height * dmap_dense_o.width) + ',')
        # log valid pixels
        logfile.write(str(countValid(dmap_dense_o.body)) + ',')
        logfile.write(str(countValid(dmap_gt_o.body)) + ',')

      # filter dmap_dense and dmap_gt bodies where 0 <= dmap_dense and 0 <= dmap_gt <= max_dist
      dmap_dense, dmap_gt = filterLists(dmap_dense_o.body, dmap_gt_o.body, min_dist, max_dist)

      if len(dmap_dense) == 0:
        continue

      if output_log:
        # log valid pixels after filtering
        logfile.write(str(countValid(dmap_dense)) + ',')
        logfile.write(str(countValid(dmap_gt)) + ',')


      # compute absolute diff between gt and dense output
      absdiff_list = np.abs(dmap_dense - dmap_gt)

      if show_time:
        print "Time before classify_near_far: ", time.time() - t

      # make data for the graphs, according to its distance to the camera (depth)
      actual_graph, bins = classify_near_far(dmap_gt, absdiff_list, bins, bin_length, min_dist)

      if show_time:
        print "Time after: ", time.time() - t

      if output_log:
        # log absolute difference valid pixels
        logfile.write(str(countValid(absdiff_list)) + ',')

      # add data to diff_list.npy (first graph)
      diff_list = addToDiffList(diff_list, absdiff_list, dmu.STEP_FIRST_GRAPH, dmu.MAXDIFF_FIRST_GRAPH)

      if show_time:
        print "Time before end: ", time.time() - t

      if output_log:
        logfile.write('\n')

      files_count += 1
      print("Processed: " + f + " - " + str(files_count) + "/" + str(files_total))

      # save data for first graph
      np.save("depth_info/diff_list.npy", diff_list[:limit])

      # save data for graphs 2-5
      # check that it is not empty data
      assert (len(bins == len(actual_graph)))
      if len(actual_graph.shape) == 1:
        np.save("depth_info/graph_depth"+str(files_count)+".npy", [bins, actual_graph])

      if len(bins) > len(max_bins):
        max_bins = bins
        np.save("depth_info/bins.npy", max_bins)


      if show_time:
        print "Iteration time: ", time.time() - t

    else:
      print "Warning, no match for file :", f



  if output_log:
    logfile.close()

def main():
  # Handle arguments
  parser = MyParser()
  parser.add_argument('dmap_dense', help='dmap dir - dense node')
  parser.add_argument('dmap_gt', help='dmap dir - ground truth')
  parser.add_argument('--output_log', help='bool - true if output a log')
  parser.add_argument('--show_time', help='bool - show computation times')
  parser.add_argument('--bin_length', help='float - length of bin in graphs')
  parser.add_argument('--max_dist', help='int - truncate depth maps using this maximum distance')
  parser.add_argument('--min_dist', help='int - truncate depth maps using this minimum distance')

  args = parser.parse_args()

  # Mandatory arguments
  if not(os.path.isdir(args.dmap_dense)) or not (os.path.isdir(args.dmap_gt)):
    print "Some of the provided dirs does not exist"
    return

  true_values = ['True', 'true', 't', '1']

  # Optional arguments
  output_log = False
  if args.output_log in true_values:
    output_log = True

  show_time = False
  if args.show_time in true_values:
    show_time = True

  min_dist = 0.0
  if (args.min_dist):
    min_dist = float(args.min_dist)

  max_dist = 0.0
  if (args.max_dist):
    max_dist = float(args.max_dist)

  bin_length = 1.0
  if (args.bin_length):
    bin_length = float(args.bin_length)

  # Make sure depth_info path exists, otherwise create it
  if not(os.path.isdir("depth_info")):
    if os.system("mkdir depth_info"):
      print "Cannot create dir depth_info"
      return

  t_orig = time.time()

  # Main process
  process(args, bin_length, min_dist, max_dist, output_log, show_time)

  print "Total Time: ", time.time() - t_orig

  # END

if __name__ == "__main__":
  main()
