# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt

def utf8(data):
  return unicode(data, 'utf-8')

# TODO: read input from file...

# Tsukuba data
data = {
	'TSUKUBA': {
		# Points that were created (triangulated) from a keyframe.
		'created': 41948977,
		# Points discarded as outliers.
		'outliers': 13990510,
		# Final point cloud size (composed by hypothesis and validated points).
		'total': 27958467,
		# Number of hypothesis in final point cloud.
		'hypothesis': 11513181,
		# Number of hypothesis in final point cloud.
		'validated': 16445286,
		# Number of matches during the sequence, i.e. number of fusions.
		'matches': 91141181,
	}
}

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
plt.show()
