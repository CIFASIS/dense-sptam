import numpy as np
import os
import matplotlib.pyplot as plt
import argparse

def np_pop(data):
    if len(data) == 0:
      print "Warning: pop from an empty list"
      return 0, []
    return data[0], data[1:]

def main():

  parser = argparse.ArgumentParser()
  parser.add_argument('bins1', help='npy - bins1')
  parser.add_argument('medians1', help='npy - medians 1')
  parser.add_argument('bins2', help='npy - bins2')
  parser.add_argument('medians2', help='npy - medians 2')
  parser.add_argument('label1', help='str label 1')
  parser.add_argument('label2', help='str label 2')
  args = parser.parse_args()

  bins1 = np.load(args.bins1)
  bins2 = np.load(args.bins2)
  medians1 = np.load(args.medians1)
  medians2 = np.load(args.medians2)

  if len(medians1) <= 0 or len(medians2) <= 0:
    print "Some of the medians are not zero.."
    return

  if len(bins1) != len(medians1) or len(bins2) != len(medians2):
    print "Bins and arrays must have the same number of dimensions to be plotted together.."
    return

  bins_extended = np.array([])
  medians1_extended = np.array([])
  medians2_extended = np.array([])
  bins1_copy = bins1
  bins2_copy = bins2
  #for i in range(max(np.max(bins1), np.max(bins2))):
  while(len(bins1_copy) != 0 and len(bins2_copy) != 0):

    if bins1_copy[0] == bins2_copy[0]:
      first_1, bins1_copy = np_pop(bins1_copy)
      first_2, bins2_copy = np_pop(bins2_copy)
      bins_extended = np.append(bins_extended, first_1)
      first, medians1 = np_pop(medians1)
      medians1_extended = np.append(medians1_extended, first)
      first, medians2 = np_pop(medians2)
      medians2_extended = np.append(medians2_extended, first)
      continue

    if(bins1_copy[0] < bins2_copy[0]):
      first, bins1_copy = np_pop(bins1_copy)
      bins_extended = np.append(bins_extended, first)
      first, medians1 = np_pop(medians1)
      medians1_extended = np.append(medians1_extended, first)
      medians2_extended = np.append(medians2_extended, 0)

    else:
      first, bins2_copy = np_pop(bins2_copy)
      bins_extended = np.append(bins_extended, first)
      first, medians2 = np_pop(medians2)
      medians2_extended = np.append(medians2_extended, first)
      medians1_extended = np.append(medians1_extended, 0)

  # add the remaining
  if len(bins1_copy) == 0:
    bins_extended = np.append(bins_extended, bins2_copy)
    medians2_extended = np.append(medians2_extended, medians2)
    medians1_extended = np.append(medians1_extended, np.zeros(len(medians2)))
  else:
    bins_extended = np.append(bins_extended, bins1_copy)
    medians1_extended = np.append(medians1_extended, medians1)
    medians2_extended = np.append(medians2_extended, np.zeros(len(medians1)))


  plt.figure(1)
  plt.plot(bins_extended, medians1_extended, label=args.label1, marker="o")
  plt.plot(bins_extended, medians2_extended, label=args.label2, marker="*")
  plt.xlabel("Distance to the camera (depth, m)")
  plt.ylabel("Median error (m)")
  plt.xticks(np.arange(min(bins_extended), max(bins_extended)+1, 5.0))
  plt.legend(loc=2)
  plt.savefig("medians_comparison.pdf", bbox_inches='tight')
  print "Saved medians_comparison.pdf"


if __name__ == "__main__":
  main()
