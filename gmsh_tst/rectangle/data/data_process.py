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
    arrs_mid_point = []
    for i in files:
        if '_' in i or ".py" in i or "png" in i:
            continue

        x = np.load(i, allow_pickle=True)
        print(x.shape)

        arrs_static.append(x)

        arrs_middle.append(np.load(i[:-4]+"_middle.npy", allow_pickle=True))
        arrs_info.append(np.load(i[:-4]+"_info.npy", allow_pickle=True))
        arrs_mid_point.append(np.load(i[:-4]+"_mid_point.npy", allow_pickle=True))


    return arrs_static, arrs_middle, np.array(arrs_info), arrs_mid_point


def plot_force_over_time(length_scale=None):

    arrs, _, info, _ = load_arrays()
    
    lengths = info[:, 0]

    colors = pl.cm.jet(np.linspace(0, 1, len(arrs)))
    
    for i, forces in enumerate(arrs):
        y_forces = forces[:, 0, :, 1]
        
        y_forces = np.sum(y_forces, axis=1)
        print(y_forces.shape)
        plt.plot(np.arange(len(y_forces)), y_forces, color=colors[i])
    plt.show()


def plot_force_over_scale_at_wall(t):
    arrs, _, info,_ = load_arrays()
    lengths = info[:, 0]
    print(lengths)
    colors = pl.cm.jet(np.linspace(0, 1, len(arrs)))

    forces = []
    for i,m in enumerate(arrs):
        print(m.shape)
        l = m[:, 0, :, 1]
        x = np.mean(l, axis=1)
        print(x.shape)
        
        plt.plot(x,  color=colors[i])
        forces.append(x)

    forces = np.array(forces)
    plt.show()
    #plt.scatter(lengths, forces)
    #plt.xlim(max(lengths), min(lengths))
    #print(np.where(info == length_scale)[0])
    #forces = arrs[np.where(info==length_scale)[0][0]]

def plot_force_over_scale_at_middle(t):
    _, _, info, arr = load_arrays()
    for i in arr:
        print("hi", i.shape)
    

    colors = pl.cm.jet(np.linspace(0, 1, len(arr)))
    pin = 20
    
    for pin in range(2002):
        i = 0
        for inf, m in zip(info, arr):
            l = m[pin, 0, 1]
            #l = np.sum(l, axis=1)
            plt.scatter(inf[0], l, color=colors[i])
            i += 1
            
        plt.savefig(str(pin)+".png")
        plt.close()
        plt.clf()
        print(pin)

    plt.show()


def plot_force_over_lenth_at_wall(t):
    arrs, _, info, _ = load_arrays()
    lengths = info[:, -1]
    print(lengths)
    colors = pl.cm.jet(np.linspace(0, 1, len(arrs)))
    
    for t in range(1000):
        forces = []
        for i, m in enumerate(arrs):
            
            l = m[t, 0, :, 1]
        
            print(l.shape)# l/(lengths[i]**1), l)
            f = lengths[i] ** -2
            x = np.sum(l)*-1
            print(x.shape)
            forces.append(x)
        
        forces = np.array(forces)
        print(forces.shape)
        plt.scatter(np.power(lengths, 0.33333)[1:], forces[1:])
        #plt.yscale("log")
        #plt.xscale("log")
        plt.xlabel("Mesh Density (number of nodes)^(1/3)")
        plt.ylabel("Total force on Static end")
        plt.title(str(t))
        #plt.show()
        plt.savefig(str(t)+".png")
        plt.clf()
        
        
    
    
    
plot_force_over_lenth_at_wall(1000)
plt.show()
exit()

#plot_force_over_scale_at_wall(800)

plt.show()
#plot_force_over_time(1.0)
