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
import pretty_boxplot as pretty_boxplot
from colors import colors




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


def plot_cloud_size(data):

	config = [
		{
			'field': 'created',
			'label': 'Puntos creados',
			'offset': 3.5,
			'color': 'b',
		},
		{
			'field': 'outliers',
			'label': 'Puntos descartados',
			'offset': 4.5,
			'color': 'g',
		},
		{
			'field': 'total',
			'label': 'Puntos finales',
			'offset': 5.5,
			'color': 'r',
		},
		{
			'field': 'hypothesis',
			'label': 'Puntos finales (hipótesis)',
			'offset': 6.7,
			'color': 'c',
		},
		{
			'field': 'validated',
			'label': 'Puntos finales (validados)',
			'offset': 7.7,
			'color': 'm',
		},
		{
			'field': 'matches',
			'label': 'Cantidad de fusiones',
			'offset': 8.9,
			'color': 'y',
		},
	]

	# Scale to millons of points
	def apply_factor(l):
		scale_factor = 1000000
		return map(lambda x: x / scale_factor, l)

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
		bar_data = apply_factor(bar_data)

		ax.bar(2 * index + offset * bar_width, bar_data, width=bar_width,
			align='edge', alpha=opacity, color=color, label=label)

	title_font = {
		'fontname':'Ubuntu', 'size':'46', 'color':'black', 'weight':'normal',
		'verticalalignment':'bottom'
	}
	axis_font = {'fontname':'Ubuntu', 'size':'46'}

	plt.xlabel('', **axis_font)
	plt.ylabel('Millones de puntos', **axis_font)
	plt.title('Resultados para el dataset Tsukuba', **title_font)

	ax.set_frame_on(False)
	plt.tick_params(axis='both', which='major', labelsize=46)

	plt.xticks(2 * index + 6 * bar_width, '', rotation=90, horizontalalignment='center')

	ax.xaxis.grid(False)
	ax.yaxis.grid(True, linewidth=5)
	#ax.set_ylim([0,140])

	plt.legend(loc='upper left', fontsize=40)

	plt.tight_layout()
	render_img(plt, 'cloud_size.png')


TASK_DISPARITY = 0;
TASK_PROJECTION = 1;
TASK_REFINEMENT = 2;
TASK_LEN = 3;

POINT_TYPE_NEW = 0;
POINT_TYPE_MATCH = 1;
POINT_TYPE_UNMATCH = 2;
POINT_TYPE_OUTLIER = 3;
POINT_TYPE_LEN = 4;

parser = argparse.ArgumentParser()
parser.add_argument('dense_log', help='dense node log')
parser.add_argument('sequence_name', help='sequence name')
parser.add_argument('--hypothesis', help='set hypothesis points number')
parser.add_argument('--validated', help='set validated points number')
parser.add_argument('--show', help='show images instead of saving', action="store_true")
args = parser.parse_args()

assert(os.path.isfile(args.dense_log))

# Output log
logfile = open(args.dense_log, "r")

task_count = [0] * TASK_LEN
task_time = [[] for i in range(TASK_LEN)]
points = [0] * POINT_TYPE_LEN

for line in logfile.readlines():

  # Omit incompleted lines, which may happen on dense node failures
  if (line[-1] != '\n'):
    continue;

  data = line.split(",")
  task = data[0]
  time = float(data[2])

  if task == 'disparity':
    task_count[TASK_DISPARITY] += 1
    task_time[TASK_DISPARITY] += [time]
  elif task == 'projection':
    task_count[TASK_PROJECTION] += 1
    task_time[TASK_PROJECTION].append(time)
    points[POINT_TYPE_NEW] += int(data[3])
    points[POINT_TYPE_MATCH] += int(data[4])
    points[POINT_TYPE_UNMATCH] += int(data[5])
    points[POINT_TYPE_OUTLIER] += int(data[6])
  elif task == 'refinement':
    task_count[TASK_REFINEMENT] += 1
    task_time[TASK_REFINEMENT].append(time)


task_time_sum = [[] for i in range(TASK_LEN)]
task_time_sum[TASK_DISPARITY] = sum(task_time[TASK_DISPARITY])
task_time_sum[TASK_PROJECTION] = sum(task_time[TASK_PROJECTION])
task_time_sum[TASK_REFINEMENT] = sum(task_time[TASK_REFINEMENT])

task_time_mean = map(lambda (x, y): (y / x) * 1000, zip(task_count, task_time_sum))

print "Keyframes processed per phase"
print "    Disparity:            " + str(task_count[TASK_DISPARITY])
print "    Heuristic/fusion:     " + str(task_count[TASK_PROJECTION])
print "    Refinement:           " + str(task_count[TASK_REFINEMENT])
print ""

print "Mean time per phase (ms)"
print "    Disparity:            " + str(task_time_mean[TASK_DISPARITY])
print "    Heuristic/fusion:     " + str(task_time_mean[TASK_PROJECTION])
print "    Refinement:           " + str(task_time_mean[TASK_REFINEMENT])
print ""

print "Heuristic results (points)"
print "    Total points created: " + str(points[POINT_TYPE_NEW])
print "    Fusions/matches:      " + str(points[POINT_TYPE_MATCH])
print "    Outliers:             " + str(points[POINT_TYPE_OUTLIER])
print ""

phase_time_data = {
	args.sequence_name: {
		'disparity': task_time_mean[TASK_DISPARITY],
		'projection': task_time_mean[TASK_PROJECTION],
		'refinement': task_time_mean[TASK_REFINEMENT],
	}
}

plot_phase_time(phase_time_data)

labels = [""] * TASK_LEN
labels[TASK_DISPARITY] = 'Disparity'
labels[TASK_PROJECTION] = 'Map Fusion and Expansion'
labels[TASK_REFINEMENT] = 'Refinement'
colors = [(0, 0.4470, 0.7410)] * TASK_LEN

pretty_boxplot.boxplot(task_time, labels, colors, "", "Time (ms)" )

cloud_size_data = {
  args.sequence_name: {
    # Points that were created (triangulated) from a keyframe.
    'created': points[POINT_TYPE_NEW],
    # Points discarded as outliers.
    'outliers': points[POINT_TYPE_OUTLIER],
    # Final point cloud size (composed by hypothesis and validated points).
    'total': int(args.hypothesis) + int(args.validated),
    # Number of hypothesis in final point cloud.
    'hypothesis': int(args.hypothesis),
    # Number of hypothesis in final point cloud.
    'validated': int(args.validated),
    # Number of matches during the sequence, i.e. number of fusions.
    'matches': points[POINT_TYPE_MATCH],
  }
}

plot_cloud_size(cloud_size_data)
