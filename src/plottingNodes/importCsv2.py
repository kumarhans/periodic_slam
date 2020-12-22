import csv
import numpy as np
import matplotlib 
font = {'family' : "Times New Roman",
        'size'   : 14}

matplotlib.rc('font', **font)
matplotlib.rc('xtick', labelsize=20) #adjust here for size of numbers
matplotlib.rc('ytick', labelsize=20) 
matplotlib.rc('axes', labelsize=20)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

t = []
gt_t_x = []
gt_t_y = []
gt_t_z = []
gt_o_x = []
gt_o_y = []
gt_o_z = []
gt_o_w = []


tvins = []
vins_t_x = []
vins_t_y = []
vins_t_z = []
vins_o_x = []
vins_o_y = []
vins_o_z = []
vins_o_w = []

firstTime = 0.0
firstTimeVins = 0.0
firstTimeOrb = 0.0




# Plot Trajectory X, Y
fig2 = plt.figure()
ax2 = fig2.add_subplot(111)

num = 5
for fil in ['est2.5f.csv','est5f.csv','est7.5f.csv','est10f.csv','est12.5f.csv','est15f.csv','est20f.csv']:

    startReading = False
    t = []
    gt_t_x = []
    gt_t_y = []
    gt_t_z = []
    gt_o_x = []
    gt_o_y = []
    gt_o_z = []
    gt_o_w = []


    tvins = []
    vins_t_x = []
    vins_t_y = []
    vins_t_z = []
    vins_o_x = []
    vins_o_y = []
    vins_o_z = []
    vins_o_w = []

    with open(fil) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            line_count += 1
            # if (line_count > 1000):
            #     break

            if not startReading:
                if np.sqrt(float(row[1])**2 + float(row[2])**2) > 0.02:
                    startReading = True
                    firstTimeVins = float(row[0])
            else:

                # Time
                offset = firstTimeVins-firstTime
                tvins.append(float(row[0])+0*offset)
                # Ground truth
                vins_t_x.append(float(row[1]))
                vins_t_y.append(float(row[2]))
                vins_t_z.append(float(row[3])-.7)
                vins_o_x.append(float(row[4]))
                vins_o_y.append(float(row[5]))
                vins_o_z.append(float(row[6])-.7)
                vins_o_w.append(float(row[7]))

                # SR-UKF SLAM2
                gt_t_x.append(float(row[8]))
                gt_t_y.append(float(row[9]))
                gt_t_z.append(float(row[10]))
                gt_o_x.append(float(row[11]))
                gt_o_y.append(float(row[12]))
                gt_o_z.append(float(row[13]))
                gt_o_w.append(float(row[14]))


                if float(row[8]) > 10:
                    break

    tvins = np.array(tvins)

    vins_t_x = np.array(vins_t_x)
    vins_t_y = np.array(vins_t_y)
    vins_t_z = np.array(vins_t_z) 
    vins_o_x = np.array(vins_o_x)
    vins_o_y = np.array(vins_o_y)
    vins_o_z = np.array(vins_o_z)
    vins_o_w = np.array(vins_o_w)

    gt_t_x = np.array(gt_t_x)
    gt_t_y = np.array(gt_t_y)
    gt_t_z = np.array(gt_t_z)
    gt_o_x = np.array(gt_o_x)
    gt_o_y = np.array(gt_o_y)
    gt_o_z = np.array(gt_o_z)
    gt_o_w = np.array(gt_o_w)

    #vins_drift = (np.sqrt((vins_t_x - gt_t_x)**2 + (vins_t_y - gt_t_y)**2 + (vins_t_z - gt_t_z)**2)) / np.sqrt((gt_t_x)**2 + (gt_t_y)**2 + (gt_t_z)**2)
    vins_drift = (np.sqrt((vins_t_x - gt_t_x)**2 + (vins_t_y - gt_t_y)**2 + (vins_t_z - gt_t_z)**2)) 

    ax2.plot( gt_t_x, vins_drift, alpha=0.5, linewidth=2.0, label='Gait Cycle Angular Rate %d' % num)


    num += 5


# Plot Drift in X, Y, Z vs Time




ax2.legend(loc='best')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('3D Drift (m)')
plt.grid(True)
plt.title("3D Drift in VINS Fusion vs GT X Distance")



plt.show()