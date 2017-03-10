import numpy as np
import matplotlib.pyplot as plt

# TODO: read input from file...
sequence_name = (
	'KITTI00', 'KITTI01', 'KITTI02', 'KITTI03', 'KITTI04', 'KITTI05',
	'KITTI06', 'KITTI07', 'KITTI08', 'KITTI09', 'KITTI10'
)

disparity_time = (
	182.973, 198.877, 203.827, 177.368, 187.058, 198.032,
	175.363, 169.038, 176.469, 177.639, 171.348
)

projection_time = (
	75.296, 66.333, 78.749, 57.480, 58.253, 73.024,
	57.659, 66.125, 65.791, 66.107, 70.559
)

refinement_time = (
	3.854, 4.866, 4.196, 3.165, 3.427, 3.966,
	3.183, 3.243, 3.067, 3.497, 3.657
)

fig, ax = plt.subplots()

index = np.arange(len(sequence_name))
bar_width = 0.5
opacity = 0.9

title_font = {
	'fontname':'Ubuntu', 'size':'24', 'color':'black', 'weight':'normal',
	'verticalalignment':'bottom'
}
axis_font = {'fontname':'Ubuntu', 'size':'18'}

rects1 = ax.bar(2 * index + 3.5 * bar_width, disparity_time, width=bar_width,
				 align='edge',
                 alpha=opacity, color='r',
                 label='Mapa de disparidad')
rects2 = ax.bar(2 * index + 4.5 * bar_width, projection_time, width=bar_width,
				 align='edge',
                 alpha=opacity, color='g',
                 label='Fusion y proyeccion 3D')

rects3 = plt.bar(2 * index + 5.5 * bar_width, refinement_time, width=bar_width,
				 align='edge',
                 alpha=opacity, color='b',
                 label='Refinamiento de posicion')

plt.xlabel('Secuencia', **axis_font)
plt.ylabel('Tiempo (ms)', **axis_font)
plt.title('Tiempo promedio por etapa', **title_font)

ax.set_frame_on(False)
plt.tick_params(axis='both', which='major', labelsize=15)

plt.xticks(2 * index + 5 * bar_width, sequence_name, rotation=90, horizontalalignment='center')
plt.legend()

ax.xaxis.grid(False)
ax.yaxis.grid(True)

plt.tight_layout()
plt.show()
