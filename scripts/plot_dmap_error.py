# coding=utf-8

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import depth_map_utilities as dmu
import glob
import matplotlib.cbook as cbook

def utf8(data):
  return unicode(data, 'utf-8')

def make_fig_1(args):
  limit = int(dmu.MAXDIFF_FIRST_GRAPH / dmu.STEP_FIRST_GRAPH)
  domain = map(lambda x: x * dmu.STEP_FIRST_GRAPH, range(0, limit))

  diff_list_file = "depth_info/diff_list.npy"
  if args.diff_list:
    diff_list_file = args.diff_list

  if not(os.path.isfile(diff_list_file)):
    print "Error: file not found: ", diff_list_file
    return

  print "Using diff list file ", diff_list_file

  # read from npy
  diff_list = np.load(diff_list_file)

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
  plt.title(utf8('Error - mapas de profundidad (dense vs. GT) para ' + args.sequence_name),
        **title_font)

  ax.set_frame_on(False)
  plt.tick_params(axis='both', which='major', labelsize=15)

  ax.set_xticks(major_ticks)
  ax.set_xticks(minor_ticks, minor=True)
  ax.set_yticks([], minor=True)

  ax.grid(which='both')
  ax.grid(which='minor', alpha=0.7)
  ax.grid(which='major', alpha=1, linewidth=2)

  plt.tight_layout()
  plt.subplots_adjust(left=0.05, right=0.95, bottom=0.1, top=0.9)


  plt.savefig(args.sequence_name+"1.png")

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('sequence_name', help='dataset sequence name')
  parser.add_argument('--diff_list', help='string - name of diff_list file (needed for Graph 1, output of depth_map.py)')
  parser.add_argument('--graph_depths', help='directory - graph files (needed for Graphs 2-5, output of depth_map.py)')
  parser.add_argument('--x_axis_spacing', help='integer - separation among ticks in the x axis (for readability))')
  args = parser.parse_args()

  make_fig_1(args)

  # depth vs errors (amount, mean)
  # load all graph_depth*.npy files

  graph_depth_dir = "./depth_info/"
  if args.graph_depths:
    graph_depth_dir = args.graph_depths

  x_axis_spacing = 5
  if args.x_axis_spacing and args.x_axis_spacing >= 1:
    x_axis_spacing = int(float(args.x_axis_spacing))


  print "Using graph depth dir: ", graph_depth_dir
  npys = glob.glob(graph_depth_dir+'graph_depth*.npy')

  if len(npys)<=0 :
    print "No data to collect.."
    return

  bins = np.load(graph_depth_dir+'bins.npy')
  print bins
  graph_depth = [[] for i in range(len(bins))]

  fig, ax = plt.subplots(1,1)
  bxpstats = list()
  graphs = []
  for npy in npys:
    g = np.load(npy)[1]
    if len(g) > 0:
      graphs.append(g)

  means = np.zeros(len(bins))
  medians = np.zeros(len(bins))
  maxs = np.zeros(len(bins))
  for i in range(len(bins)):
    graph_depth = []

    for j in range(len(npys)):
      if i < len(graphs[j]):
        if len(graphs[j][i]) > 0:
          graph_depth.extend(graphs[j][i])

    if len(graph_depth) > 0:
      means[i] = np.mean(graph_depth)
      medians[i] = np.median(graph_depth)
      maxs[i] = np.max(graph_depth)
      bxpstats.extend(cbook.boxplot_stats(np.ravel(graph_depth)))
    else:
      bxpstats.extend(cbook.boxplot_stats(np.ravel([0])))

    print "ITEM : " , i, len(graph_depth)

  ax.bxp(bxpstats, showfliers=False)
  bins_str = map(lambda x: str(bins[x]) if x % x_axis_spacing == 0 else '', range(len(bins)))

  # bins-bins[0]+1 since it can start at any number
  plt.xticks(bins-bins[0]+1, bins_str)
  plt.xlabel("Distance to the camera (depth, m)")
  plt.ylabel("Error (m)")
  plt.savefig(args.sequence_name+"2.png")

  # mean
  plt.figure(3)
  plt.plot(bins, means)
  plt.xlabel("Distance to the camera (depth, m)")
  plt.ylabel("Mean error (m)")
  plt.savefig(args.sequence_name+"3.png")

  plt.figure(4)
  plt.plot(bins, medians)
  plt.xlabel("Distance to the camera (depth, m)")
  plt.ylabel("Median error (m)")
  plt.savefig(args.sequence_name+"4.png")

  plt.figure(5)
  plt.plot(bins, maxs)
  plt.xlabel("Distance to the camera (depth, m)")
  plt.ylabel("Max error (m)")
  plt.savefig(args.sequence_name+"5.png")

  print ""
  print "Saved " + args.sequence_name + "{1-5}.png files"



if __name__ == "__main__":
  main()
