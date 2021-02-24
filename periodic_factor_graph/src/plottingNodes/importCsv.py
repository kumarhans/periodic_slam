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

est_t_x = []
est_t_y = []
est_t_z = []
est_o_x = []
est_o_y = []
est_o_z = []
est_o_w = []

tvins = []
vins_t_x = []
vins_t_y = []
vins_t_z = []
vins_o_x = []
vins_o_y = []
vins_o_z = []
vins_o_w = []

torb = []
orb_t_x = []
orb_t_y = []
orb_t_z = []
orb_o_x = []
orb_o_y = []
orb_o_z = []
orb_o_w = []


firstTime = 0.0
firstTimeVins = 0.0
firstTimeOrb = 0.0


startReading = False

with open('periodicIMU_VO.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        line_count += 1
        # if (line_count > 1000):
        #     break

        if not startReading:
            if np.sqrt(float(row[1])**2 + float(row[2])**2) > 0.02:
                startReading = True
                firstTime = float(row[0])
        else:

            # Time
            t.append(float(row[0]))
            # Ground truth
            est_t_x.append(float(row[1]))
            est_t_y.append(float(row[2]))
            est_t_z.append(float(row[3]))
            est_o_x.append(float(row[4]))
            est_o_y.append(float(row[5]))
            est_o_z.append(float(row[6]))
            est_o_w.append(float(row[7]))

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

startReading = False

with open('vinsSplit.csv') as csv_file:
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

            # # SR-UKF SLAM2
            # gt_t_x.append(float(row[8]))
            # gt_t_y.append(float(row[9]))
            # gt_t_z.append(float(row[10]))
            # gt_o_x.append(float(row[11]))
            # gt_o_y.append(float(row[12]))
            # gt_o_z.append(float(row[13]))
            # gt_o_w.append(float(row[14]))

            if float(row[8]) > 10:
                break


startReading = False

with open('vinsSplitNot.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        line_count += 1
        # if (line_count > 1000):
        #     break

        if not startReading:
            if np.sqrt(float(row[1])**2 + float(row[2])**2) > 0.02:
                startReading = True
                firstTimeOrb = float(row[0])
        else:

            # Time
            offset = firstTimeOrb-firstTime
            torb.append(float(row[0])+0*offset)
            # Ground truth
            orb_t_x.append(float(row[1]))
            orb_t_y.append(float(row[2]))
            orb_t_z.append(float(row[3])-.7)
            orb_o_x.append(float(row[4]))
            orb_o_y.append(float(row[5]))
            orb_o_z.append(float(row[6]))
            orb_o_w.append(float(row[7]))

            # # SR-UKF SLAM2
            # gt_t_x.append(float(row[8]))
            # gt_t_y.append(float(row[9]))
            # gt_t_z.append(float(row[10]))
            # gt_o_x.append(float(row[11]))
            # gt_o_y.append(float(row[12]))
            # gt_o_z.append(float(row[13]))
            # gt_o_w.append(float(row[14]))

            if float(row[8]) > 10:
                break



#Calculate Distance Traveled Vector
# gt_DistanceTravledX = [0]
# gt_DistanceTravledY = [0]
# gt_DistanceTravledZ = [0]

# est_DistanceTravledX = [0]
# est_DistanceTravledY = [0]
# est_DistanceTravledZ = [0]




# for index in range(1,len(t)):
#     dt = t[index] - t[index-1]
#     dpos_x = abs(gt_t_x[index] - gt_t_x[index-1])
#     dpos_y = abs(gt_t_y[index] - gt_t_y[index-1])
#     dpos_z = abs(gt_t_z[index] - gt_t_z[index-1])
#     gt_DistanceTravledX.append(gt_DistanceTravledX[index-1] + dpos_x)
#     gt_DistanceTravledY.append(gt_DistanceTravledY[index-1] + dpos_y)
#     gt_DistanceTravledZ.append(gt_DistanceTravledZ[index-1] + dpos_z)

#     dpos_x = abs(ukf_t_x[index] - ukf_t_x[index-1])
#     dpos_y = abs(ukf_t_y[index] - ukf_t_y[index-1])
#     dpos_z = abs(ukf_t_z[index] - ukf_t_z[index-1])
#     ukf_DistanceTravledX.append(ukf_DistanceTravledX[index-1] + dpos_x)
#     ukf_DistanceTravledY.append(ukf_DistanceTravledY[index-1] + dpos_y)
#     ukf_DistanceTravledZ.append(ukf_DistanceTravledZ[index-1] + dpos_z)

#     dpos_x = abs(vins_t_x[index] - vins_t_x[index-1])
#     dpos_y = abs(vins_t_y[index] - vins_t_y[index-1])
#     dpos_z = abs(vins_t_z[index] - vins_t_z[index-1])
#     vins_DistanceTravledX.append(vins_DistanceTravledX[index-1] + dpos_x)
#     vins_DistanceTravledY.append(vins_DistanceTravledY[index-1] + dpos_y)
#     vins_DistanceTravledZ.append(vins_DistanceTravledZ[index-1] + dpos_z)

#     # vx = gt_t_vx[index]
    # gt_DistanceTravledX.append(gt_DistanceTravledX[index-1] + abs(vx*dt))

    # vy = gt_t_vy[index]
    # gt_DistanceTravledY.append(gt_DistanceTravledY[index-1] + abs(vy*dt))

    # vz = gt_t_vz[index]
    # gt_DistanceTravledZ.append(gt_DistanceTravledZ[index-1] + abs(vz*dt))





t = np.array(t)

gt_t_x = np.array(gt_t_x)
gt_t_y = np.array(gt_t_y)
gt_t_z = np.array(gt_t_z)
gt_o_x = np.array(gt_o_x)
gt_o_y = np.array(gt_o_y)
gt_o_z = np.array(gt_o_z)
gt_o_w = np.array(gt_o_w)

est_t_x = np.array(est_t_x)
est_t_y = np.array(est_t_y)
est_t_z = np.array(est_t_z) 
est_o_x = np.array(est_o_x)
est_o_y = np.array(est_o_y)
est_o_z = np.array(est_o_z)
est_o_w = np.array(est_o_w)

tvins = np.array(tvins)

vins_t_x = np.array(vins_t_x)
vins_t_y = np.array(vins_t_y)
vins_t_z = np.array(vins_t_z) 
vins_o_x = np.array(vins_o_x)
vins_o_y = np.array(vins_o_y)
vins_o_z = np.array(vins_o_z)
vins_o_w = np.array(vins_o_w)


torb = np.array(torb)

orb_t_x = np.array(orb_t_x)
orb_t_y = np.array(orb_t_y)
orb_t_z = np.array(orb_t_z) 
orb_o_x = np.array(orb_o_x)
orb_o_y = np.array(orb_o_y)
orb_o_z = np.array(orb_o_z)
orb_o_w = np.array(orb_o_w)

# gt_DistanceTravledX = np.array(gt_DistanceTravledX)
# gt_DistanceTravledY = np.array(gt_DistanceTravledY)
# gt_DistanceTravledZ = np.array(gt_DistanceTravledZ)

# ukf_DistanceTravledX = np.array(ukf_DistanceTravledX)
# ukf_DistanceTravledY = np.array(ukf_DistanceTravledY)
# ukf_DistanceTravledZ = np.array(ukf_DistanceTravledZ)

# vins_DistanceTravledX = np.array(vins_DistanceTravledX)
# vins_DistanceTravledY = np.array(vins_DistanceTravledY)
# vins_DistanceTravledZ = np.array(vins_DistanceTravledZ)

# gt_t_x = -1 * np.array(gt_t_x)
# gt_t_y = -1 * np.array(gt_t_y)
# gt_t_z = -1 * np.array(gt_t_z)
# gt_o_x = -1 * np.array(gt_o_x)
# gt_o_y = -1 * np.array(gt_o_y)
# gt_o_z = -1 * np.array(gt_o_z)
# gt_o_w = -1 * np.array(gt_o_w)
#
# ukf_t_x = -1 * np.array(ukf_t_x)
# ukf_t_y = -1 * np.array(ukf_t_y)
# ukf_t_z = -1 * np.array(ukf_t_z)
# ukf_o_x = -1 * np.array(ukf_o_x)
# ukf_o_y = -1 * np.array(ukf_o_y)
# ukf_o_z = -1 * np.array(ukf_o_z)
# ukf_o_w = -1 * np.array(ukf_o_w)
#
# vins_t_x = -1 * np.array(vins_t_x)
# vins_t_y = -1 * np.array(vins_t_y)
# vins_t_z = -1 * np.array(vins_t_z)
# vins_o_x = -1 * np.array(vins_o_x)
# vins_o_y = -1 * np.array(vins_o_y)
# vins_o_z = -1 * np.array(vins_o_z)
# vins_o_w = -1 * np.array(vins_o_w)

# Plot in Distance Traveled vs Time
#@Shuo use this as template
# fig0 = plt.figure()
# ax0 = fig0.add_subplot(111)
# ax0.plot(t, np.sqrt((gt_DistanceTravledX**2 + gt_DistanceTravledY**2 + gt_DistanceTravledZ**2)), 'b-', label='Distance', alpha=0.5,linewidth=2.0)
# # fig0.xlim(0, 10)
# # fig0.ylim(0, 10)
# ax0.legend(loc='best', prop={'size': 6})
# ax0.set_xlabel('Time (s)', fontsize = 50)
# ax0.set_ylabel('Distance(m)', fontsize = 50)
# plt.grid(True)
# plt.title("Distance Travled vs time")
# # plt.show()
# plt.savefig('DistTravled.eps',format='eps', dpi=300)







# Plot Trajectory X, Y, Z
# fig1 = plt.figure()
# ax1 = fig1.add_subplot(111, projection='3d')
# ax1.plot(gt_t_x, gt_t_y, gt_t_z, 'r-', label='Ground Truth', alpha=.5, linewidth=2.0)
# ax1.plot(ukf_t_x, ukf_t_y, ukf_t_z, 'b-', label='SR-UKF', alpha=0.5, linewidth=2.0)
# ax1.plot(vins_t_x, vins_t_y, vins_t_z, 'g-', label='VINS-Fusion', alpha=0.5, linewidth=2.0)
# ax1.legend(loc='best')
# ax1.set_xlabel('X (m)')
# ax1.set_ylabel('Y (m)')
# ax1.set_zlabel('Z (m)')
# #ax1.set_xlim([0,4])
# #ax1.set_ylim([-0.5,0.5])
# #ax1.set_zlim([-0.25,0.25])
# ax1.axis('equal')
# plt.grid(True)
# # plt.title("Ground Truth vs SR-UKF vs VINS-Fusion")
# # plt.show()
# plt.savefig('comparison3D.eps',format='eps', dpi=300)

# Plot Trajectory X, Y
fig2 = plt.figure()
ax2 = fig2.add_subplot(111)
ax2.plot(gt_t_x, gt_t_y, 'r-', label='Ground Truth', alpha=.5,linewidth=2.0)
ax2.plot(est_t_x, est_t_y, 'b-', label='periodic SLAM', alpha=0.5, linewidth=2.0)
ax2.plot(vins_t_x, vins_t_y, 'g-', label='VINS-Fusion Only Down Frames', alpha=0.5, linewidth=2.0)
ax2.plot(orb_t_x, orb_t_y, 'y-', label='VINS-Fusion', alpha=0.5, linewidth=2.0)
#ax2.plot(vins_t_x, vins_t_y, 'g-', label='VINS-Fusion', alpha=0.5, linewidth=2.0)
ax2.legend(loc='best')
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
#ax2.set_xlim([-2.0,.5])
ax2.set_ylim([-2,2])
ax2.axis('equal')
plt.grid(True)
plt.title("XY Trajectories")
# plt.show()
plt.savefig('comparison2D.png',format='png', dpi=300)






# # Plot Drift in X, Y, Z vs Time
# ukf_drift = (np.sqrt((ukf_t_x - gt_t_x)**2 + (ukf_t_y - gt_t_y)**2 + (ukf_t_z - gt_t_z)**2)) / np.sqrt((gt_t_x)**2 + (gt_t_y)**2 + (gt_t_z)**2)
# vins_drift = (np.sqrt((vins_t_x - gt_t_x)**2 + (vins_t_y - gt_t_y)**2 + (vins_t_z - gt_t_z)**2)) / np.sqrt((gt_t_x)**2 + (gt_t_y)**2 + (gt_t_z)**2)
# fig3 = plt.figure()
# ax3 = fig3.add_subplot(111)
# ax3.plot(t, ukf_drift, 'b-', label='periodic', alpha=0.5,linewidth=2.0)
# ax3.plot(t, vins_drift, 'g-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)
# ax3.legend(loc='best')
# ax3.set_xlabel('Time (s)')
# ax3.set_ylabel('Drift percentage (%)')
# plt.grid(True)
# plt.title("Drift percentage (3D) in periodic and VINS-Fusion vs Time")
# # plt.show()
# plt.savefig('driftPercent3D.eps',format='eps', dpi=300)



# Plot Drift in X, Y vs Time
# ukf_drift = (np.sqrt((ukf_t_x - gt_t_x)**2 + (ukf_t_y - gt_t_y)**2)) / np.sqrt((gt_t_x)**2 + (gt_t_y)**2)
# vins_drift = (np.sqrt((vins_t_x - gt_t_x)**2 + (vins_t_y - gt_t_y)**2)) / np.sqrt((gt_t_x)**2 + (gt_t_y)**2)

# print(vins_drift[-1])

# fig4 = plt.figure()
# ax4 = fig4.add_subplot(111)
# ax4.plot(t, ukf_drift, 'b-', label='periodic', alpha=0.5,linewidth=2.0)
# ax4.plot(t, vins_drift, 'g-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)
# ax4.legend(loc='best')
# ax4.set_xlabel('Time (s)')
# ax4.set_ylabel('Drift percentage (%)')
# plt.grid(True)
# plt.title("Drift percentage (2D) in periodic and VINS-Fusion vs Time")
# # plt.show()
# plt.savefig('driftPercent2D.eps',format='eps', dpi=300)







# Plot Error in X, Y, Z vs Time
# ukf_drift = np.sqrt((ukf_t_x - gt_t_x)**2 + (ukf_t_y - gt_t_y)**2 + (ukf_t_z - gt_t_z)**2)
# vins_drift = np.sqrt((vins_t_x - gt_t_x)**2 + (vins_t_y - gt_t_y)**2 + (vins_t_z - gt_t_z)**2)

# fig3 = plt.figure()
# ax3 = fig3.add_subplot(111)
# ax3.plot(t, ukf_drift, 'b-', label='periodic', alpha=0.5,linewidth=2.0)
# ax3.plot(t, vins_drift, 'g-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)
# ax3.legend(loc='best')
# ax3.set_xlabel('Time (s)')
# ax3.set_ylabel('Euclidean drift (m)')
# plt.grid(True)
# plt.title("Euclidean drift (3D) in periodic and VINS-Fusion vs time")
# # plt.show()
# plt.savefig('Error3D.eps',format='eps', dpi=300)



# Plot Error in X, Y vs Time
# ukf_drift = np.sqrt((ukf_t_x - gt_t_x)**2 + (ukf_t_y - gt_t_y)**2)
# vins_drift = np.sqrt((vins_t_x - gt_t_x)**2 + (vins_t_y - gt_t_y)**2)

# fig4 = plt.figure()
# ax4 = fig4.add_subplot(111)
# ax4.plot(t, ukf_drift, 'b-', label='periodic', alpha=0.5,linewidth=2.0)
# ax4.plot(t, vins_drift, 'g-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)
# ax4.legend(loc='best')
# ax4.set_xlabel('Time (s)')
# ax4.set_ylabel('Euclidean drift (m)')
# plt.grid(True)
# plt.title("Euclidean drift (2D) in periodic and VINS-Fusion vs time")
# # plt.show()
# plt.savefig('Error2D.eps',format='eps', dpi=300)










# Plot in X Drift vs Time
# fig5 = plt.figure()
# ax5 = fig5.add_subplot(111)

# ax5.plot(t[3:], abs(gt_DistanceTravledX[3:]-ukf_DistanceTravledX[3:])/gt_DistanceTravledX[3:] , 'b-', label='periodic', alpha=0.5,linewidth=2.0)
# ax5.plot(t[3:], abs(gt_DistanceTravledX[3:]-vins_DistanceTravledX[3:])/gt_DistanceTravledX[3:] , 'g-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)

# ax5.legend(loc='best')
# ax5.set_xlabel('Time (s)')
# ax5.set_ylabel('X Drift')
# plt.grid(True)
# plt.title("X Axis Position Drift of periodic and VINS-Fusion vs time")
# # plt.show()
# plt.savefig('PosDriftx.eps',format='eps', dpi=300)



# Plot in Y Drift vs Time
# fig6 = plt.figure()
# ax6 = fig6.add_subplot(111)
# ax6.plot(t[3:], abs(gt_DistanceTravledY[3:]-ukf_DistanceTravledY[3:])/gt_DistanceTravledY[3:] , 'b-', label='periodic', alpha=0.5,linewidth=2.0)
# ax6.plot(t[3:], abs(gt_DistanceTravledY[3:]-vins_DistanceTravledY[3:])/gt_DistanceTravledY[3:] , 'g-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)

# ax6.legend(loc='best')
# ax6.set_xlabel('Time (s)')
# ax6.set_ylabel('Y Drift')
# plt.grid(True)
# plt.title("Y Axis Position Drift of periodic and VINS-Fusion vs time")
# # plt.show()
# plt.savefig('PosDrifty.eps',format='eps', dpi=300)


# Plot in Z Drift vs Time
# fig7 = plt.figure()
# ax7 = fig7.add_subplot(111)
# ax7.plot(t[3:], abs(gt_DistanceTravledZ[3:]-ukf_DistanceTravledZ[3:])/gt_DistanceTravledZ[3:] , 'b-', label='periodic', alpha=0.5,linewidth=2.0)
# ax7.plot(t[3:], abs(gt_DistanceTravledZ[3:]-vins_DistanceTravledZ[3:])/gt_DistanceTravledZ[3:] , 'g-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)

# ax7.legend(loc='best')
# ax7.set_xlabel('Time (s)')
# ax7.set_ylabel('Z drift')
# plt.grid(True)
# # plt.title("Z Axis Position Drift of periodic and VINS-Fusion vs time")
# # plt.show()
# plt.savefig('PosDriftz.eps',format='eps', dpi=300)








# # Plot Drift in X, Y, Z Velocity vs Time

# ukf_drift = np.sqrt((ukf_t_vx - gt_t_vx)**2 + (ukf_t_vy - gt_t_vy)**2 + (ukf_t_vz - gt_t_vz)**2)

# vins_drift = np.sqrt((vins_t_vx - gt_t_vx)**2 + (vins_t_vy - gt_t_vy)**2 + (vins_t_vz - gt_t_vz)**2)

# fig3 = plt.figure()
# ax3 = fig3.add_subplot(111)
# ax3.plot(t, ukf_drift, 'b-', label='periodic', alpha=0.5,linewidth=2.0)
# #ax3.plot(t, vins_drift, 'g-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)
# ax3.legend(loc='best')
# ax3.set_xlabel('Time (s)')
# ax3.set_ylabel('Euclidean drift (m)')
# plt.grid(True)
# plt.title("Euclidean Velocity drift (3D) in periodic and VINS-Fusion vs time")
# # plt.show()
# plt.savefig('driftVel3D.eps',format='eps', dpi=300)

# # Plot in X, Y Velocity vs Time

# ukf_drift = np.sqrt((ukf_t_vx - gt_t_vx)**2 + (ukf_t_vy - gt_t_vy)**2)

# vins_drift = np.sqrt((vins_t_vx - gt_t_vx)**2 + (vins_t_vy - gt_t_vy)**2)

# fig4 = plt.figure()
# ax4 = fig4.add_subplot(111)
# ax4.plot(t, ukf_drift, 'b-', label='periodic SLAM', alpha=0.5,linewidth=2.0)
# #ax4.plot(t, vins_drift, 'g-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)
# ax4.legend(loc='best')
# ax4.set_xlabel('Time (s)')
# ax4.set_ylabel('Euclidean drift (m)')
# plt.grid(True)
# plt.title("Euclidean Velocity drift (2D) in periodic SLAM and VINS-Fusion vs time")
# # plt.show()
# plt.savefig('driftVel2D.eps',format='eps', dpi=300)

# Plot in X Pos vs Time
fig8 = plt.figure()
ax8 = fig8.add_subplot(111)
ax8.plot(t, gt_t_x, 'r-', label='Ground Truth', alpha=0.5,linewidth=2.0)
ax8.plot(t, est_t_x, 'b-', label='periodic SLAM', alpha=0.5,linewidth=2.0)
ax8.plot(tvins, vins_t_x, 'g-', label='VINS-Fusion Only Down Frames', alpha=0.5,linewidth=2.0)
ax8.plot(torb, orb_t_x, 'y-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)

ax8.legend(loc='best')
ax8.set_xlabel('Time (s)')
ax8.set_ylabel('X Position (m)')
ax8.set_ylim([0,11])
plt.grid(True)
plt.title("X Axis Position vs Time")
# plt.show()
plt.savefig('Posx.png',format='png', dpi=300)



# Plot in Y Pos vs Timein
fig9 = plt.figure()
ax9 = fig9.add_subplot(111)
ax9.plot(t, gt_t_y, 'r-', label='Ground Truth', alpha=0.5,linewidth=2.0)
ax9.plot(t, est_t_y, 'b-', label='periodic SLAM', alpha=0.5,linewidth=2.0)
ax9.plot(tvins, vins_t_y, 'g-', label='VINS-Fusion Only Down Frames', alpha=0.5,linewidth=2.0)
ax9.plot(torb, orb_t_y, 'y-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)

ax9.legend(loc='best')
ax9.set_xlabel('Time (s)')
ax9.set_ylabel('Y Position (m)')
ax9.set_ylim([-1.5,1.5])
plt.grid(True)

plt.title("Y Axis Position vs Time")
# plt.show()
plt.savefig('Posy.png',format='png', dpi=300)

# Plot in Z Pos vs Timein
fig95 = plt.figure()
ax95 = fig95.add_subplot(111)
ax95.plot(t, gt_t_z, 'r-', label='Ground Truth', alpha=0.5,linewidth=2.0)
ax95.plot(t, est_t_z, 'b-', label='periodic SLAM', alpha=0.5,linewidth=2.0)
ax95.plot(tvins, vins_t_z, 'g-', label='VINS-Fusion Only Down Frames', alpha=0.5,linewidth=2.0)
ax95.plot(torb, orb_t_z, 'y-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)

ax95.legend(loc='best')
ax95.set_xlabel('Time (s)')
ax95.set_ylabel('Z Position (m)')
ax95.set_ylim([0,2.5])
plt.grid(True)
plt.title("Z Axis Position vs Time")
#plt.show()
plt.savefig('Posz.png',format='png', dpi=300)
plt.show()


startIdx = 4000
endIdx = 6000


# Plot in X Velocity vs Time
# print(t.shape)
# fig10 = plt.figure()
# ax10 = fig10.add_subplot(111)
# ax10.plot(t[startIdx:endIdx], gt_t_vx[startIdx:endIdx], 'r-', label='Ground Truth', alpha=0.5,linewidth=1.0)
# ax10.plot(t[startIdx:endIdx], ukf_t_vx[startIdx:endIdx], 'b-', label='SR-UKF', alpha=0.5,linewidth=1.0)
# ax10.plot(t[startIdx:endIdx], vins_t_vx[startIdx:endIdx], 'g-', label='VINS-Fusion', alpha=0.5,linewidth=1.0)

# ax10.legend(loc='best')
# ax10.set_xlabel('Time (s)')
# ax10.set_ylabel('X Velocity (m/s)')
# plt.grid(True)
# plt.title("X Axis Velocity of SR-UKF, VINS-Fusion, and Ground Truth vs time")
# # plt.show()
# plt.savefig('Velx.eps',format='eps', dpi=300)



# # Plot in Y Velocity vs Time
# fig11 = plt.figure()
# ax11 = fig11.add_subplot(111)
# ax11.plot(t[startIdx:endIdx], gt_t_vy[startIdx:endIdx], 'r-', label='Ground Truth', alpha=0.5,linewidth=1.0)
# ax11.plot(t[startIdx:endIdx], ukf_t_vy[startIdx:endIdx], 'b-', label='SR-UKF', alpha=0.5,linewidth=1.0)
# ax11.plot(t[startIdx:endIdx], vins_t_vy[startIdx:endIdx], 'g-', label='VINS-Fusion', alpha=0.5,linewidth=1.0)

# ax11.legend(loc='best')
# ax11.set_xlabel('Time (s)')
# ax11.set_ylabel('Y Velocity (m/s)')
# plt.grid(True)
# plt.title("Y Axis Velocity of SR-UKF, VINS-Fusion, and Ground Truth vs time")
# # plt.show()
# plt.savefig('Vely.eps',format='eps', dpi=300)


# # Plot in Z Velocity vs Time
# fig111 = plt.figure()
# ax111 = fig111.add_subplot(111)
# ax111.plot(t[startIdx:endIdx], gt_t_vz[startIdx:endIdx], 'r-', label='Ground Truth', alpha=0.5,linewidth=3.0)
# ax111.plot(t[startIdx:endIdx], ukf_t_vz[startIdx:endIdx], 'b-', label='SR-UKF', alpha=0.5,linewidth=3.0)
# ax111.plot(t[startIdx:endIdx], vins_t_vz[startIdx:endIdx], 'g-', label='VINS-Fusion', alpha=0.5,linewidth=3.0)

# ax111.legend(loc='best')
# ax111.set_xlabel('Time (s)')
# ax111.set_ylabel('Z Velocity (m/s)')
# plt.grid(True)
# # plt.title("Z Axis Velocity of SR-UKF, VINS-Fusion, and Ground Truth vs Time")
# plt.show()
# plt.savefig('Velz.eps',format='eps', dpi=300)






# Plot in X Velocity Error vs Time
# fig12 = plt.figure()
# ax12 = fig12.add_subplot(111)
# ax12.plot(t, gt_t_vx-ukf_t_vx, 'b-', label='SR-UKF', alpha=0.5,linewidth=2.0)
# ax12.plot(t, gt_t_vx-vins_t_vx, 'g-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)
# ax12.legend(loc='best')
# ax12.set_xlabel('Time (s)')
# ax12.set_ylabel('Velocity Error (m/s)')
# plt.grid(True)
# plt.title("X Velocity Error in SR-UKF and VINS-Fusion vs time")
# plt.savefig('VelxErr.eps',format='eps', dpi=300)


# # Plot in Y Velocity Error vs Time
# fig13 = plt.figure()
# ax13 = fig13.add_subplot(111)
# ax13.plot(t, gt_t_vx-ukf_t_vy, 'b-', label='SR-UKF', alpha=0.5,linewidth=2.0)
# ax13.plot(t, gt_t_vx-vins_t_vy, 'g-', label='VINS-Fusion', alpha=0.5,linewidth=2.0)
# ax13.legend(loc='best')
# ax13.set_xlabel('Time (s)')
# ax13.set_ylabel('Velocity Error (m/s)')
# plt.grid(True)
# plt.title("Y Velocity Error in SR-UKF and VINS-Fusion vs time")
# plt.savefig('VelyErr.eps',format='eps', dpi=300)






# # Plot Drift in X vs Time
#
# ukf_drift = np.sqrt((ukf_t_x - gt_t_x)**2)
#
# vins_drift = np.sqrt((vins_t_x - gt_t_x)**2)
#
# fig4 = plt.figure()
# ax4 = fig4.add_subplot(111)
# ax4.plot(t, ukf_drift, 'b-', label='SR-UKF SLAM', alpha=0.5)
# ax4.plot(t, vins_drift, 'g-', label='VINS-Fusion', alpha=0.5)
# ax4.legend(loc='best')
# ax4.set_xlabel('Timestep')
# ax4.set_ylabel('Euclidean distance')
# plt.grid(True)
# plt.title("Drift in X in SR-UKF SLAM and VINS-Fusion with respect to time")
# # plt.show()
# plt.savefig('driftX.eps',format='eps', dpi=300)
#
# # Plot Drift in Y vs Time
#
# ukf_drift = np.sqrt((ukf_t_y - gt_t_y)**2)
#
# vins_drift = np.sqrt((vins_t_y - gt_t_y)**2)
#
# fig4 = plt.figure()
# ax4 = fig4.add_subplot(111)
# ax4.plot(t, ukf_drift, 'b-', label='SR-UKF SLAM', alpha=0.5)
# ax4.plot(t, vins_drift, 'g-', label='VINS-Fusion', alpha=0.5)
# ax4.legend(loc='best')
# ax4.set_xlabel('Timestep')
# ax4.set_ylabel('Euclidean distance')
# plt.grid(True)
# plt.title("Drift in Y in SR-UKF SLAM and VINS-Fusion with respect to time")
# # plt.show()
# plt.savefig('driftY.eps',format='eps', dpi=300)
#
# # Plot Drift in Z vs Time
#
# ukf_drift = np.sqrt((ukf_t_z - gt_t_z)**2)
#
# vins_drift = np.sqrt((vins_t_z - gt_t_z)**2)
#
# fig4 = plt.figure()
# ax4 = fig4.add_subplot(111)
# ax4.plot(t, ukf_drift, 'b-', label='SR-UKF SLAM', alpha=0.5)
# ax4.plot(t, vins_drift, 'g-', label='VINS-Fusion', alpha=0.5)
# ax4.legend(loc='best')
# ax4.set_xlabel('Timestep')
# ax4.set_ylabel('Euclidean distance')
# plt.grid(True)
# plt.title("Drift in Z in SR-UKF SLAM and VINS-Fusion with respect to time")
# # plt.show()
# plt.savefig('driftZ.eps',format='eps', dpi=300)
