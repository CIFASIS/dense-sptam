import numpy as np
import matplotlib.pyplot as plt

# TODO: read input from file...
sequence_name = [
	'KITTI00', 'KITTI01', 'KITTI02', 'KITTI03', 'KITTI04', 'KITTI05',
	'KITTI06', 'KITTI07', 'KITTI08', 'KITTI09', 'KITTI10',
]

total_points = [
	312364860,
	62326865,
	416882968,
	32458357,
	20359902,
	169359821,
	76589756,
	58978284,
	248366690,
	118222233,
	79559358,
]

merged_points = [
	417456754,
	25609042,
	544811606,
	40085724,
	19007203,
	222875300,
	90162619,
	83929963,
	352192785,
	141410385,
	108375069,
]

outliers_points = [
	178842199,
	16598632,
	169222060,
	15867089,
	8661135,
	86339248,
	35269139,
	35022408,
	147149858,
	57486058,
	37514186,
]

# Tsukuba data

#sequence_name = [ 'TSUKUBA' ]
#total_points = [ 29372276 ]
#match_points = [ 83006220 ]
#merged_points = [ 12607734 ]
#outliers_points = [ 16820314 ]

scale_factor = 1000000

def apply_factor(l):
	return map(lambda x: x / scale_factor, l)

total_points = apply_factor(total_points)
match_points = apply_factor(match_points)
merged_points = apply_factor(merged_points)
outliers_points = apply_factor(outliers_points)

fig, ax = plt.subplots()

index = np.arange(len(sequence_name))
bar_width = 0.5
opacity = 0.9

title_font = {
	'fontname':'Ubuntu', 'size':'24', 'color':'black', 'weight':'normal',
	'verticalalignment':'bottom'
}
axis_font = {'fontname':'Ubuntu', 'size':'18'}

rects1 = ax.bar(2 * index + 3.5 * bar_width, total_points, width=bar_width,
				 align='edge',
                 alpha=opacity, color='b',
                 label='Total de puntos')

rects2 = ax.bar(2 * index + 4.5 * bar_width, merged_points, width=bar_width,
				 align='edge',
                 alpha=opacity, color='g',
                 label='Puntos fusionados')

rects3 = ax.bar(2 * index + 5.5 * bar_width, match_points, width=bar_width,
				 align='edge',
                 alpha=opacity, color='y',
                 label='Total de fusiones')

rects4 = ax.bar(2 * index + 6.5 * bar_width, outliers_points, width=bar_width,
				 align='edge',
                 alpha=opacity, color='r',
                 label='Puntos descartados')

plt.xlabel('Secuencia', **axis_font)
plt.ylabel('Millones de puntos', **axis_font)
plt.title('', **title_font)

ax.set_frame_on(False)
plt.tick_params(axis='both', which='major', labelsize=15)

plt.xticks(2 * index + 6 * bar_width, '', rotation=90, horizontalalignment='center')
plt.legend()

ax.xaxis.grid(False)
ax.yaxis.grid(True)

plt.tight_layout()
plt.show()
