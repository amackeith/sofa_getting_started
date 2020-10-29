import numpy as np
import matplotlib.pyplot as plt
import os, sys
import matplotlib.pylab as pl


def load_arrays():
    files = os.listdir(".")
    files.sort()
    files.reverse()
    print(files)

    arrs_static = []
    arrs_middle = []
    arrs_info = []
    for i in files:
        if '_' in i or ".py" in i:
            continue

        x = np.load(i, allow_pickle=True)
        print(x.shape)

        arrs_static.append(x)

        arrs_middle.append(np.load(i[:-4]+"_middle.npy", allow_pickle=True))
        arrs_info.append(np.load(i[:-4]+"_info.npy", allow_pickle=True))

    return arrs_static, arrs_middle, np.array(arrs_info)


def plot_force_over_time(length_scale):

    arrs, _, info = load_arrays()
    print(info[np.where(info == length_scale)[0]])
    print(np.where(info==length_scale)[0])
    forces = arrs[np.where(info==length_scale)[0][0]]


    for i in range(3):
        y_forces = forces[:, 0, :, i]
        y_forces = np.mean(y_forces, axis=1)
        print(y_forces.shape)
        plt.plot(np.arange(len(y_forces)), y_forces)
    plt.show()


def plot_force_over_scale_at_wall(t):
    arrs, _, info = load_arrays()
    lengths = info[:, 0]
    print(lengths)

    forces = []
    for m in arrs:
        l = m[:, 0, :, 1]


        forces.append(np.mean(l))

    forces = np.array(forces)

    plt.scatter(lengths, forces)
    #plt.xlim(max(lengths), min(lengths))
    #print(np.where(info == length_scale)[0])
    #forces = arrs[np.where(info==length_scale)[0][0]]

def plot_force_over_scale_at_middle(t):
    arr, _, info = load_arrays()
    print('hi', len(arr))
    colors = pl.cm.jet(np.linspace(0, 1, len(arr)))

    for i, m in enumerate(arr):
        l = m[:, 0, :, 2]
        l = np.sum(l, axis=1)
        plt.plot(l, color=colors[i])

    plt.show()

        

plot_force_over_scale_at_middle(800)
exit()

#plot_force_over_scale_at_wall(800)

plt.show()
#plot_force_over_time(1.0)
