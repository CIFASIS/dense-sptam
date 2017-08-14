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

def listToArray(data, height, width):
  ret = np.zeros((height, width))
  for i in range(0, height):
    for j in range(0, width):
      ret[i, j] = data[i * width + j]
  return ret

def arrayToList(arr):
  return reduce(operator.add, arr.tolist())

def applyMask(l, mask):
  assert(len(l) == len(mask))
  l_mask = zip(l, mask)
  return map(lambda (x, y): x if y else -1, l_mask)

def filterList(l, limit):
  return map(lambda x: x if x <= limit else -1, l)

def countValid(l):
  return len(filter(lambda x: x >= 0, l))

def intersectMask(dmap_x, dmap_y):
  assert(dmap_x.shape[0] == dmap_y.shape[0])
  assert(dmap_x.shape[1] == dmap_y.shape[1])
  ret = np.zeros(dmap_x.shape[0], dmap_x.shape[1])
  for i in range(0, dmap_x.shape[0]):
    for j in range(0, dmap_x.shape[1]):
      if (dmap_x[i, j] >= 0 and dmap_y[i, j] >= 0):
        ret[i, j] = 1
  return ret

def absoluteDiffList(l_x, l_y):
  assert(len(l_x) == len(l_y))
  return map(lambda (a, b): abs(a - b) if a >= 0 and b >= 0 else -1, zip(l_x, l_y))

INVALID_DEPTH = -10;

def absoluteDiffMap(dmap_x, dmap_y):
  assert(dmap_x.shape[0] == dmap_y.shape[0])
  assert(dmap_x.shape[1] == dmap_y.shape[1])
  ret = np.full((dmap_x.shape[0], dmap_x.shape[1]), INVALID_DEPTH)
  for i in range(0, dmap_x.shape[0]):
    for j in range(0, dmap_x.shape[1]):
      if (dmap_x[i, j] >= 0 and dmap_y[i, j] >= 0):
        ret[i, j] = abs(dmap_x[i, j] - dmap_y[i, j])
  return ret

def getRGBcolor01(val, high, hsv_s = 0.8, hsv_v = 0.8, factor = 1):
  hsv = ((1 - float(val) / float(high)) * factor, hsv_s, hsv_v)
  return colorsys.hsv_to_rgb(*hsv)

def getRGBcolor(val, high, hsv_s = 0.8, hsv_v = 0.8, factor = 1):
  return tuple(int(i * 255) for i in getRGBcolor01(val, high, hsv_s, hsv_v, factor))

def genRGBcolors(N):
  return map(lambda x: getRGBcolor(x, N), range(N))

def doColorArray(arr, limit = 0):
  background = (127, 127, 127)
  if (not limit):
    limit = np.amax(arr)
  img = np.full((arr.shape[0], arr.shape[1], 3), background, np.uint8)
  for i in range(0, arr.shape[0]):
    for j in range(0, arr.shape[1]):
      if (arr[i, j] >= 0):
        img[i, j] = getRGBcolor(arr[i, j], limit)
  return img

def saveImage(filename, img):
  cv2.imwrite(filename, img)

def plotSaveImage(filename, img):
  imgplot = plt.imshow(img)
  plt.colorbar()
  plt.savefig(filename, dpi=300)

class DepthMap:

  def __init__(self, filename):
    self.filename = filename
    dmapfile = open(self.filename, "r")
    self.parseheader(dmapfile.readline())
    self.parsebody(dmapfile.readline())
    dmapfile.close()

  def parseheader(self, line):
    data = line.split(",")
    self.height = int(data[0])
    self.width = int(data[1])

  def parsebody(self, line):
    self.body = map(float, line.split(",")[:self.height * self.width])
    self.body = map(lambda x: INVALID_DEPTH if x < 0 else x, self.body)

  def getnparray(self):
    return listToArray(self.body, self.height, self.width)

parser = argparse.ArgumentParser(epilog='NOTE: Invalid (negative) depth values are mapped to ' + str(INVALID_DEPTH))
parser.add_argument('dmap', help='dmap file')
parser.add_argument('--compare', help='dmap file to compare')
parser.add_argument('--save', help='save image to output file')
parser.add_argument('--clim_low', help='plotter clim low bound')
parser.add_argument('--clim_high', help='plotter clim high bound')
args = parser.parse_args()

assert(os.path.isfile(args.dmap))
dmap_orig = DepthMap(args.dmap)
dmap_orig_arr = dmap_orig.getnparray()

if args.compare:
  assert(os.path.isfile(args.compare))
  dmap_compare = DepthMap(args.compare)
  dmap_compare_arr = dmap_compare.getnparray()
  dmap_orig_arr = absoluteDiffMap(dmap_orig_arr, dmap_compare_arr)

fig, ax = plt.subplots()
ax.set_axis_off()

clim_low = int(args.clim_low) if args.clim_low else None
clim_high = int(args.clim_high) if args.clim_high else None

# Different color maps can be used to plot the image:
# NICE: nipy_spectral
# UGLY: gist_stern, gist_earth, jet
# NOTE: append _r at the end of the cmap name to reverse it

imgplot = plt.imshow(dmap_orig_arr, clim = (clim_low, clim_high), cmap=plt.get_cmap('nipy_spectral'))
plt.colorbar(orientation ='horizontal', label = "Distance to the camera (depth, m)")

if args.save:
	filename = args.save
	plt.savefig(filename)
	print('Image saved to ' + filename)
else:
	plt.show(imgplot)
