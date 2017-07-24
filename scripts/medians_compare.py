import numpy as np
import os
import matplotlib.pyplot as plt
import argparse

def main():

  parser = argparse.ArgumentParser()
  parser.add_argument('bins', help='npy - bins')
  parser.add_argument('medians1', help='npy - medians 1')
  parser.add_argument('medians2', help='npy - medians 2')
  args = parser.parse_args()

  bins = np.load(args.bins)
  medians1 = np.load(args.medians1)
  medians2 = np.load(args.medians2)

  if len(medians1) <= 0 or len(medians2) <= 0:
    print "Some of the medians are not zero.."
    return

  if len(bins) != len(medians1) or len(bins) != len(medians2):
    print "Arrays must have the same number of dimensions to be plotted together.."
    return

  plt.figure(1)
  plt.plot(bins, medians1, label="Medians 1", marker="o")
  plt.plot(bins, medians2, label="Medians 2", marker="*")
  plt.xlabel("Distance to the camera (depth, m)")
  plt.ylabel("Median error (m)")
  plt.legend(loc=2)
  plt.savefig("medians_comparison.png")



if __name__ == "__main__":
  main()
