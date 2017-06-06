
def simplePlot(data):
	plt.plot(data)
	plt.show()

def boxPlot(data):
	plt.boxplot(data, 0, 'bo')
	plt.show()

def histogramPlot(data, bin_factor):
	bins = int(np.amax(data)) * bin_factor
	plt.hist(data, bins)
	plt.show()

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

def filterMap(dmap, limit):
	for i in range(0, dmap.shape[0]):
		for j in range(0, dmap.shape[1]):
			if (dmap[i, j] > limit):
				dmap[i, j] = -1

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


def absoluteDiffMap(dmap_x, dmap_y):
	assert(dmap_x.shape[0] == dmap_y.shape[0])
	assert(dmap_x.shape[1] == dmap_y.shape[1])
	ret = np.full((dmap_x.shape[0], dmap_x.shape[1]), -1)
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
	plt.imshow(img)
	plt.savefig(filename, dpi=300)
