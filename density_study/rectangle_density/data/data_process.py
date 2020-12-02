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



def tmp():
    _, _, info, _ = load_arrays()
    print(info)

tmp()
exit()


def plot_force_over_time(length_scale=None):


    arrs, _, info, _ = load_arrays()
    
    p = 8
    arrs = arrs[p:]
    info = info[p:]
    lengths = info[:, -1]
    time_ratio = info[:, 1] / 2
    arrs = arrs[:]
    print(time_ratio)
    exit()
    forces = []
    time_ratios = []
    times = []
    for i, m in enumerate(arrs):
        l = m[:, 0, :, 1]

        print(l.shape)
        # f = lengths[i] ** -2
        x = np.sum(l, axis=1) * -1
        print("x.shape", x.shape)
        forces.extend(x)
        time_ratios.extend(np.ones(x.shape)*time_ratio[i])
        times.extend(np.arange(len(x)))

    forces = np.array(forces)
    time_ratios = np.array(time_ratios)
    times = np.array(times)*0.001
    print(forces.shape, times.shape, time_ratios.shape)
    plt.scatter(times, forces, c=time_ratios, norm=cl.LogNorm(), s=0.5)
    plt.show()
    exit()

    print(forces.shape)


    #plt.xlabel("Mesh Density (number of nodes)^(1/3)")
    #plt.ylabel("Total force on Static end")
    #plt.title("Force at fixed end time= " + str(t * 0.001) + " sec")
    cb = plt.colorbar()
    cb.set_label("Compute Time:Simulation Time Ratio")
    # plt.show()
    #plt.savefig("ForceFixedEnd" + str(t).zfill(3) + ".png")
    plt.clf()

plot_force_over_time()
exit()



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
    for t in range(100):
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

        plt.scatter(np.power(lengths, 1.0/3.0), forces, c=time_ratio,
                    norm=cl.LogNorm())
        #plt.yscale("log")
        #plt.xscale("log")
        plt.xlabel("Mesh Density (number of nodes)^(1/3)")
        plt.ylabel("Total force on Static end")
        plt.title("Force at fixed end time= "+ str(t*0.001) + " sec")
        cb = plt.colorbar()
        cb.set_label("Compute Time:Simulation Time Ratio")
        #plt.show()
        plt.savefig("ForceFixedEnd" + str(t).zfill(3)+".png")
        plt.clf()
        
        
    
    
    
<<<<<<< HEAD:gmsh_tst/rectangle_density/data/data_process.py
#plot_force_over_lenth_at_wall(1000)
#plt.show()
#exit()
=======
#plot_force_over_lenth_at_wall(200)
#exit()

def scale_vs_density():
    _, _, info, _ = load_arrays()

    info = info[8:]
    print(info[:, 0])
    fig, ax = plt.subplots()
    im = ax.scatter(info[:,0], np.power(info[:, -1], 1.0/3.0),  c=info[:, 1]/2,norm=cl.LogNorm())
    cb = fig.colorbar(im)
    cb.set_label("Compute Time : Simulation Time Ratio")

    ax.set_ylabel("Mesh Density (number of nodes)^(1/3)")
    #alt ax.set_ylabel("number of nodes")

    ax.set_xlabel("Length Scale in Gmsh (clscale)")

    ax.set_yscale("log")
    ax.set_xscale("log")

    fig.suptitle("Mesh length scale vs density with compute time")
    #ax.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    plt.savefig("Scale_density_compute_time_alt_zoom.png")
    plt.show()
>>>>>>> 6a5f4ad6954ce8beeb4a887ae39fdd5997ff43ff:gmsh_tst/rectangle/data/data_process.py

#scale_vs_density()
#exit()

<<<<<<< HEAD:gmsh_tst/rectangle_density/data/data_process.py
#plt.show()
#plot_force_over_time(1.0)
=======
>>>>>>> 6a5f4ad6954ce8beeb4a887ae39fdd5997ff43ff:gmsh_tst/rectangle/data/data_process.py
