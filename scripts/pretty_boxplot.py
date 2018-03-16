import itertools
import matplotlib.pyplot as plt

def splitLabels( labels ):
  new_labels = []
  for label in labels:
    new_labels.append( '\n'.join( label.split(' / ') ).upper() )
  return new_labels

def boxplot( datas, labels, colors, title, axis ):
  """
  @param datas
    iterable container containing N data sets (arrays), each to be condensed
    in a boxplot to be plotted side by side.
  @param labels
    iterable containing N labels (strings), one for each data set.
  @param colors
    iterable containing N colors (3-uple of floats in the range 0-1 representing RGB values).
  @param title
    plot title (string).
  @param axis
    y-axis label (string).
  """

  # check if labels are strings
  if type(labels[0]) is str:
    labels = splitLabels( labels )

  fig = plt.figure()
  ax = fig.add_subplot(111)

  bp = ax.boxplot( datas, patch_artist=True )

  # tomo la cantidad de colores que necesito (uno por cada tipo de dato)
  colors = colors[:len(labels)]

  ## change outline color, fill color and linewidth of the boxes
  for box, label, color in zip( bp['boxes'], labels, colors ):

    box.name = label

    # change outline color
    box.set(color=color, linewidth=2)

    # change fill color
    box.set(facecolor = color + (0.5,))

  ## change color and linewidth of the medians
  for median, color in zip(bp['medians'], colors):
    median.set(color=color, linewidth=2)

  ## change color and linewidth of the whiskers
  for whisker, color in zip(bp['whiskers'], itertools.chain.from_iterable([item, item] for item in colors)):
    whisker.set(color=color, linewidth=2)
  
  ## change color and linewidth of the caps
  for cap, color in zip(bp['caps'], itertools.chain.from_iterable([item, item] for item in colors)):
    cap.set(color=color, linewidth=2)

  ## change the style of fliers and their fill

  # sometimes if the outliers are just on one side ov the avg,
  # the number of flier arrays is one per data sample, instead of two.

  # make sure it is one of the cases we can handle. Either all samples
  # have one flier array, or all have two.
  assert( len(datas)==len( bp['fliers'] ) or 2*len(datas)==len( bp['fliers'] ) )
  __colors__ = colors if len(datas)==len( bp['fliers'] ) else itertools.chain.from_iterable([item, item] for item in colors)

  for flier, color in zip(bp['fliers'], __colors__):
    flier.set(marker='o', color=color, alpha=0.5)

  ## set labels
  plt.setp(ax, xticklabels=labels)

  ax.set_ylabel( axis )

  if title:
    fig.suptitle( title )

  return fig
