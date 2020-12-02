import numpy as np
import matplotlib.pyplot as plt
import os, sys
import matplotlib.pylab as pl
import matplotlib.colors as cl
import matplotlib

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
    
    p = 8

    arrs = arrs[p:]
    info = info[p:]
    lengths = info[:, -1]
    time_ratio = info[:, 1] / 2
    arrs = arrs[:]

    forces = []
    time_ratios = []
    times = []
    for i, m in enumerate(arrs):
        l = m[40:100, 0, :, 1]

        print(l.shape)
        # f = lengths[i] ** -2
        x = np.sum(l, axis=1) * -1
        print("x.shape", x.shape)
        forces.extend(x)
        time_ratios.extend(np.ones(x.shape)*time_ratio[i])
        times.extend(np.arange(len(x)))

    forces = np.array(forces)
    time_ratios = np.array(time_ratios)
    times = np.array(times)*0.001+0.04
    #print(forces.shape, times.shape, time_ratios.shape)
    plt.scatter(times, forces, c=time_ratios,  cmap=None, norm=cl.LogNorm(), s=0.5)


    print(forces.shape)


    #plt.xlabel("Mesh Density (number of nodes)^(1/3)")
    #plt.ylabel("Total force on Static end")
    #plt.title("Force at fixed end time= " + str(t * 0.001) + " sec")
    cb = plt.colorbar()
    cb.set_label("Compute Time:Simulation Time Ratio")
    plt.ylabel("Total Force on Fixed side")
    plt.xlabel("Time (sec)")
    plt.title("Phase Graph Zoom")
    #plt.savefig("PhaseGraph_small_2.png")
    plt.show()


#plot_force_over_time()
#exit()

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
    p = 8
    arrs = arrs[p:]
    info = info[p:]
    lengths = info[:, -1]
    time_ratio = info[:, 1] / 2
    arrs = arrs[:]
    print(lengths)


    #exit()
    for t in [400,]:
        forces = []
        for i, m in enumerate(arrs):
            
            l = m[t, 0, :, 1]
        
            print(l.shape)
            #f = lengths[i] ** -2
            x = np.sum(l)*-1
            print(x.shape)
            forces.append(x)
        
        forces = np.array(forces)
        print(forces.shape)
        x = np.power(lengths, 1.0 / 3.0)
        y = forces
        plt.scatter(x,y, c=time_ratio,
                    norm=cl.LogNorm(), cmap=None)

        ax = plt.gca()
        for i in range(len(time_ratio)):
            print(i)
            if not i in [13, 15, 17, 19, 23]:
                continue
            s = "{:.1e}".format(time_ratio[i])
            s = s[:]
            ax.annotate(s, (x[i], y[i]))
        #plt.yscale("log")
        #plt.xscale("log")
        plt.xlabel("Mesh Density (number of nodes)^(1/3)")
        plt.ylabel("Total force on Static end")
        plt.title("Force at fixed end time= "+ str(t*0.001) + " sec")
        cb = plt.colorbar()
        cb.set_label("Compute Time:Simulation Time Ratio")
        plt.show()
        #plt.savefig("ForceFixedEnd" + str(t).zfill(3)+".png")
        plt.clf()
        
        
    
    
    
plot_force_over_lenth_at_wall(200)
exit()

def scale_vs_density():
    _, _, info, _ = load_arrays()
    plt.loglog(info[:, 1], info[:,0])
    plt.show()
    exit()
    info = info[8:]
    num_nodes = info[:, -1]
    length_scale = info[:,0]
    time_ratio = info[:, 1]/2

    fig, ax = plt.subplots()

    num_nodes = np.power(num_nodes, 1.0/3.0)
    im = ax.scatter(length_scale, num_nodes,
                    c=time_ratio,cmap=None , norm=cl.LogNorm())
    cb = fig.colorbar(im)
    cb.set_label("Compute Time : Simulation Time Ratio")

    ax.set_ylabel("Mesh Density (number of nodes)^(1/3)")
    #alt ax.set_ylabel("number of nodes")

    ax.set_xlabel("Length Scale in Gmsh (clscale)")

    #ax.set_yscale("log")
    #ax.set_xscale("log")

    fig.suptitle("Mesh length scale vs density with compute time")
    #ax.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ax = plt.gca()
    for i in range(len(time_ratio)):
        print(i)
        if not i in [14,16, 18, 20, 24,26]:
            continue
        s = "{:.1e}".format(time_ratio[i])
        s = s[:]
        ax.annotate(s, (length_scale[i], num_nodes[i]))
    plt.savefig("Scale_density_compute_time_annotated.png")
    #plt.show()

scale_vs_density()
exit()

