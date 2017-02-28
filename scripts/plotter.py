import matplotlib.pyplot as plt
import numpy as np

diff_list_file = open("diff_list.txt", "r")

header = diff_list_file.readline().split(',')
step = float(header[0])
maxdiff = float(header[1])
limit = int(maxdiff / step)

diff_list = diff_list_file.readline().split(',')
diff_list = map(int, diff_list[:limit])

plt.hist(diff_list)
plt.title("Gaussian Histogram")
plt.xlabel("Value")
plt.ylabel("Frequency")
plt.show()
