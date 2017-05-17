# coding=utf-8

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os

def utf8(data):
	return unicode(data, 'utf-8')

parser = argparse.ArgumentParser()
parser.add_argument('diff_list', help='diff_list file - output from depth_map.py')
parser.add_argument('sequence_name', help='dataset sequence name')
args = parser.parse_args()

assert(os.path.isfile(args.diff_list))
diff_list_file = open(args.diff_list, "r")
sequence_name = args.sequence_name

#TODO: don't hardcode this values...
MAXDIFF = 100.0
STEP = 0.1

limit = int(MAXDIFF / STEP)
domain = map(lambda x: x * STEP, range(0, limit))

diff_list = diff_list_file.readline().split(',')
diff_list = map(int, diff_list[:limit])

# Discard 0 values at the end
diff_list_max = 0
for i in range(0, len(diff_list)):
	if diff_list[i] > 0:
		diff_list_max = i + 1
diff_list = diff_list[0:diff_list_max]

major_ticks = np.arange(0, float(diff_list_max) * STEP, 1)
minor_ticks = np.arange(0, float(diff_list_max) * STEP, STEP)

fig, ax = plt.subplots()

index = np.arange(len(diff_list))
bar_width = 0.10
opacity = 0.7

title_font = {
	'fontname':'Ubuntu', 'size':'18', 'color':'black', 'weight':'normal',
	'verticalalignment':'bottom'
}
axis_font = {'fontname':'Ubuntu', 'size':'18'}

rects1 = ax.bar(index * STEP, diff_list, width=bar_width,
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
plt.show()

# depth vs errors (amount, mean)
import numpy as np
import matplotlib.pyplot as plt


a = np.load("graph_depth.npy")

# amount
#plt.plot(a[0], a[1])

print(map(lambda x: np.median(x) if len(x)>0 else 0, a[1]))
# mean
plt.plot(a[0], map(lambda x: np.mean(x) if len(x)>0 else 0, a[1]))
plt.xlabel("Distance to the camera (depth, m)")
plt.ylabel("Mean error (m)")
plt.show()

plt.plot(a[0], map(lambda x: np.median(x) if len(x)>0 else 0, a[1]))
plt.xlabel("Distance to the camera (depth, m)")
plt.ylabel("Median error (m)")
plt.show()


plt.plot(a[0], map(lambda x: np.max(x) if len(x)>0 else 0, a[1]))
plt.xlabel("Distance to the camera (depth, m)")
plt.ylabel("Max error (m)")
plt.show()
