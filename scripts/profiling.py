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

TASK_DISPARITY = 0;
TASK_STEREOSCAN = 1;
TASK_PROJECTION = 2;
TASK_REFINEMENT = 3;

STEREOSCAN_NEW = 0;
STEREOSCAN_MATCH = 1;
STEREOSCAN_UNMATCH = 2;
STEREOSCAN_OUTLIER = 3;

parser = argparse.ArgumentParser()
parser.add_argument('dense_log', help='dense node log')
args = parser.parse_args()

assert(os.path.isfile(args.dense_log))

# Output log
logfile = open(args.dense_log, "r")

task_count = [0] * 4
task_time = [0.0] * 4
stereoscan_points = [0] * 5

for line in logfile.readlines():
  data = line.split(",")
  task = data[0]
  time = float(data[2])

  if task == 'disparity':
    task_count[TASK_DISPARITY] += 1
    task_time[TASK_DISPARITY] += time
  elif task == 'stereoscan':
    task_count[TASK_STEREOSCAN] += 1
    task_time[TASK_STEREOSCAN] += time
    stereoscan_points[STEREOSCAN_NEW] += int(data[3])
    stereoscan_points[STEREOSCAN_MATCH] += int(data[4])
    stereoscan_points[STEREOSCAN_UNMATCH] += int(data[5])
    stereoscan_points[STEREOSCAN_OUTLIER] += int(data[6])
  elif task == 'projection':
    task_count[TASK_PROJECTION] += 1
    task_time[TASK_PROJECTION] += time
  elif task == 'refinement':
    task_count[TASK_REFINEMENT] += 1
    task_time[TASK_REFINEMENT] += time

assert(task_count[TASK_STEREOSCAN] == task_count[TASK_PROJECTION])

task_time_mean = map(lambda (x, y): (y / x) * 1000, zip(task_count, task_time))

print "Keyframes processed per phase"
print "    Disparity:            " + str(task_count[TASK_DISPARITY])
print "    Heuristic/fusion:     " + str(task_count[TASK_STEREOSCAN])
print "    Refinement:           " + str(task_count[TASK_REFINEMENT])
print ""

print "Mean time per phase (ms)"
print "    Disparity:            " + str(task_time_mean[TASK_DISPARITY])
print "    Heuristic/fusion:     " + str(task_time_mean[TASK_STEREOSCAN] + task_time_mean[TASK_PROJECTION])
print "    Refinement:           " + str(task_time_mean[TASK_REFINEMENT])
print ""

print "Heuristic results (points)"
print "    Total points created: " + str(stereoscan_points[STEREOSCAN_NEW])
print "    Fusions/matches:      " + str(stereoscan_points[STEREOSCAN_MATCH])
print "    Outliers:             " + str(stereoscan_points[STEREOSCAN_OUTLIER])
print ""
