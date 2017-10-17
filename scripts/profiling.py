# coding=utf-8

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

def utf8(data):
	return unicode(data, 'utf-8')

def render_img(plt, filename):
	if (args.show):
		plt.show()
	else:
		plt.savefig(filename)

def plot_phase_time(data):

	config = [
		{
			'field': 'disparity',
			'label': 'Cómputo de disparidad',
			'offset': 3.5,
			'color': 'b',
		},
		{
			'field': 'projection',
			'label': 'Expansión y fusión de mapa',
			'offset': 4.5,
			'color': 'g',
		},
		{
			'field': 'refinement',
			'label': 'Refinamiento de mapa',
			'offset': 5.5,
			'color': 'r',
		},
	]

	fig, ax = plt.subplots()

	index = np.arange(len(data))
	bar_width = 0.2
	opacity = 0.9

	for field_config in config:
		field_name = field_config['field']
		label = utf8(field_config['label'])
		offset = field_config['offset']
		color = field_config['color']

		bar_data = [sequence_data[field_name] for sequence_name, sequence_data in data.items()]

		ax.bar(2 * index + offset * bar_width, bar_data, width=bar_width,
			align='edge', alpha=opacity, color=color, label=label)

	title_font = {
		'fontname':'Ubuntu', 'size':'24', 'color':'black', 'weight':'normal',
		'verticalalignment':'bottom'
	}
	axis_font = {'fontname':'Ubuntu', 'size':'18'}

	plt.xlabel('', **axis_font)
	plt.ylabel('Tiempo (ms)', **axis_font)
	plt.title('Tiempo promedio por etapa para el dataset Tsukuba', **title_font)

	ax.set_frame_on(False)
	plt.tick_params(axis='both', which='major', labelsize=15)

	plt.xticks(2 * index + 6 * bar_width, '', rotation=90, horizontalalignment='center')

	ax.xaxis.grid(False)
	ax.yaxis.grid(True)

	plt.legend()

	plt.tight_layout()
	render_img(plt, 'phase_time.png')


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
parser.add_argument('sequence_name', help='sequence name')
parser.add_argument('--show', help='show images instead of saving', action="store_true")
args = parser.parse_args()

assert(os.path.isfile(args.dense_log))

# Output log
logfile = open(args.dense_log, "r")

task_count = [0] * 4
task_time = [0.0] * 4
stereoscan_points = [0] * 5

for line in logfile.readlines():

  # Omit incompleted lines, which may happen on dense node failures
  if (line[-1] != '\n'):
    continue;

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

phase_time_data = {
	args.sequence_name: {
		'disparity': task_time_mean[TASK_DISPARITY],
		'projection': task_time_mean[TASK_STEREOSCAN] + task_time_mean[TASK_PROJECTION],
		'refinement': task_time_mean[TASK_REFINEMENT],
	}
}

plot_phase_time(phase_time_data)
