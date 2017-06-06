# coding=utf-8

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import depth_map_utilities as dmu


def utf8(data):
	return unicode(data, 'utf-8')

parser = argparse.ArgumentParser()
parser.add_argument('diff_list', help='diff_list file - output from depth_map.py')
#parser.add_argument('graph_depth', help='graph_depth file - output from depth_map.py')
parser.add_argument('sequence_name', help='dataset sequence name')
args = parser.parse_args()

assert(os.path.isfile(args.diff_list))
sequence_name = args.sequence_name

limit = int(dmu.MAXDIFF_FIRST_GRAPH / dmu.STEP_FIRST_GRAPH)
domain = map(lambda x: x * dmu.STEP_FIRST_GRAPH, range(0, limit))

# read from npy
diff_list = np.load(args.diff_list)

# Discard 0 values at the end
diff_list_max = 0
for i in range(0, len(diff_list)):
	if diff_list[i] > 0:
		diff_list_max = i + 1
diff_list = diff_list[0:diff_list_max]

major_ticks = np.arange(0, float(diff_list_max) * dmu.STEP_FIRST_GRAPH, 1)
minor_ticks = np.arange(0, float(diff_list_max) * dmu.STEP_FIRST_GRAPH, dmu.STEP_FIRST_GRAPH)

fig, ax = plt.subplots()

index = np.arange(len(diff_list))
bar_width = 0.10
opacity = 0.7

title_font = {
	'fontname':'Ubuntu', 'size':'18', 'color':'black', 'weight':'normal',
	'verticalalignment':'bottom'
}
axis_font = {'fontname':'Ubuntu', 'size':'18'}

rects1 = ax.bar(index * dmu.STEP_FIRST_GRAPH, diff_list, width=bar_width,
				 align='edge', log=True,
                 alpha=opacity, color='b', edgecolor='b')

plt.xlabel('Error (metros)', **axis_font)
plt.ylabel(utf8('Cantidad de píxeles'), **axis_font)
plt.title(utf8('Error entre mapas de profundidad (método vs. ground truth) para la secuencia ' + sequence_name),
		  **title_font)

ax.set_frame_on(False)
plt.tick_params(axis='both', which='major', labelsize=15)

ax.set_xticks(major_ticks)
ax.set_xticks(minor_ticks, minor=True)
ax.set_yticks([], minor=True)

plt.legend()

ax.grid(which='both')
ax.grid(which='minor', alpha=0.7)
ax.grid(which='major', alpha=1, linewidth=2)

plt.tight_layout()
plt.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.9)

# depth vs errors (amount, mean)
# load all graph_depth*.npy files
import glob
npys = glob.glob('./graph_depth*.npy')

assert(len(npys)>0)
a = np.load(npys[0])
bins = a[0]
graph_depth = []
for i in range(len(bins)):
	graph_depth.append([])



for npy in npys:
	actual_graph = np.load(npy)[1]
	for i in range(len(bins)):
		graph_depth[i].extend(actual_graph[i])


fig = plt.figure(2)
ax = fig.add_subplot(111)
bp = ax.boxplot(graph_depth)

plt.xlabel("Distance to the camera (depth, m)")
plt.ylabel("Error (m)")


# mean
plt.figure(3)
plt.plot(bins, map(lambda x: np.mean(x) if len(x)>0 else 0, graph_depth))
plt.xlabel("Distance to the camera (depth, m)")
plt.ylabel("Mean error (m)")

plt.figure(4)
plt.plot(bins, map(lambda x: np.median(x) if len(x)>0 else 0, graph_depth))
plt.xlabel("Distance to the camera (depth, m)")
plt.ylabel("Median error (m)")

plt.figure(5)
plt.plot(bins, map(lambda x: np.max(x) if len(x)>0 else 0, graph_depth))
plt.xlabel("Distance to the camera (depth, m)")
plt.ylabel("Max error (m)")


plt.show()
