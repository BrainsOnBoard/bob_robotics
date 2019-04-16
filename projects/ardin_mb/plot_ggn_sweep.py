import numpy as np

from glob import glob
from os import path
from sys import argv

import matplotlib.pyplot as plt

mean_num_kc = []
var_num_kc = []
titles = []
for f in glob(argv[1] + "/*.csv"):
    sweep_title = path.splitext(path.basename(f))[0]

    sweep_params = [float(p) for p in sweep_title.split("_")]
    assert len(sweep_params) == 4

    data = np.loadtxt(f, delimiter=",", skiprows=1,
                      dtype={"names": ("num_pn", "num_kc"),
                             "formats": (np.int, np.int)})

    titles.append("KC->GGN weight:%f\nGGN->KC weight:%f\nGGN->KC VMid:%f\nGGN->KC VSlope:%f" % tuple(sweep_params))

    mean_num_kc.append(np.average(data["num_kc"]))
    var_num_kc.append(np.std(data["num_kc"]) / mean_num_kc[-1])

fig, axis = plt.subplots()
scatter = axis.scatter(mean_num_kc, var_num_kc, s=2)

axis.set_xlabel("Mean")
axis.set_ylabel("Coefficient of variance")

annot = axis.annotate("", xy=(0,0), xytext=(20,20),textcoords="offset points",
                      bbox=dict(boxstyle="round", fc="w"),
                      arrowprops=dict(arrowstyle="->"))
annot.set_visible(False)

def update_annot(ind):
    pos = scatter.get_offsets()[ind["ind"][0]]
    annot.xy = pos
    annot.set_text("\n\n".join(titles[n] for n in ind["ind"]))
    annot.get_bbox_patch().set_alpha(0.4)


def hover(event):
    vis = annot.get_visible()
    if event.inaxes == axis:
        cont, ind = scatter.contains(event)
        if cont:
            update_annot(ind)
            annot.set_visible(True)
            fig.canvas.draw_idle()
        else:
            if vis:
                annot.set_visible(False)
                fig.canvas.draw_idle()

fig.canvas.mpl_connect("motion_notify_event", hover)
plt.show()
    
