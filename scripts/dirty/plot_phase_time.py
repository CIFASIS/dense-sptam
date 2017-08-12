# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt

def utf8(data):
  return unicode(data, 'utf-8')

# TODO: read input from file...

# Tsukuba data
data = {
	'TSUKUBA': {
		'disparity': 118.805,
		'projection': 144.828,
		'refinement': 4.358,
	}
}

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
plt.show()
