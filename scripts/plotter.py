#!/usr/bin/env python
import matplotlib.pyplot as plt
import csv


# ========== set up necessary parts ========== #
COMMON_PATH = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/'
PLOT_PATH = '/home/mingi/catkin_ws/src/object_detection_cs169/plot/'
B = 'case2.csv'
C = 'case3.csv'
D = 'case4.csv'

B_CPA = 'case2_CPA.csv'
C_CPA = 'case3_CPA.csv'
D_CPA = 'case4_CPA.csv'

# PATH_A = COMMON_PATH + A
PATH_B = COMMON_PATH + B
PATH_C = COMMON_PATH + C
PATH_D = COMMON_PATH + D

PATH_B_CPA = COMMON_PATH + B_CPA
PATH_C_CPA = COMMON_PATH + C_CPA
PATH_D_CPA = COMMON_PATH + D_CPA

GROUND_TRUTH_B = 0.4
GROUND_TRUTH_C = 0.6
GROUND_TRUTH_D = 0.4

NAME = ['case2: straight walk', 'case3: running', 'case4: drunken driving']
LINE_WIDTH = 1

x = []
y = []
x2 =[]
y2 = []
fig = plt.figure()
# plt by calling saved data from each path
with open(PATH_B,'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        x.append(row[0])
        y.append(float(row[1]))

plt.plot(x,y, color='b', label ='case2: straight walk', lw= LINE_WIDTH)
plt.title('case2: straight walk', fontsize=20)

#plt.plot(x,y, color='g', label ='case3: running', lw= LINE_WIDTH)
#plt.title('case3: running', fontsize=20)

#plt.plot(x,y, color='r', label ='case4: drunken driving', lw= LINE_WIDTH)
#plt.title('case4: drunken driving', fontsize=20)

plt.xlabel('Time [sec]', fontsize=15)
plt.ylabel('Distance [m]', fontsize=15)
# plt.legend(loc = 'lower right', fontsize=10)
plt.xlim(0, 15)
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)

fig.savefig(PLOT_PATH+'case2.png')

# ========================= figure 1 =========================================
fig2 = plt.figure()

with open(PATH_B_CPA,'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        x2.append(row[0])
        y2.append(float(row[1]))

plt.plot(x2,y2, color='b', label ='case2: straight walk', lw= LINE_WIDTH)
plt.plot([0, x2[-1]], [GROUND_TRUTH_B, GROUND_TRUTH_B], label ='ground truth', color='k', LineStyle='--', lw= LINE_WIDTH+5)
plt.title('case2: straight walk', fontsize=20)

#plt.plot(x2,y2, color='g', label ='case3: running', lw= LINE_WIDTH)
#plt.plot([0, x2[-1]], [GROUND_TRUTH_C, GROUND_TRUTH_C], label ='ground truth', color='k', LineStyle='--', lw= LINE_WIDTH+5)
#plt.title('case3: running', fontsize=20)

#plt.plot(x2,y2, color='r', label ='case4: drunken driving', lw= LINE_WIDTH)
#plt.plot([0, x2[-1]], [GROUND_TRUTH_D, GROUND_TRUTH_D],label ='ground truth', color='k', LineStyle='--', lw= LINE_WIDTH+5)
#plt.title('case4: drunken driving', fontsize=20)

plt.xlabel('Time [sec]', fontsize=15)
plt.ylabel('CPA [m]', fontsize=15)
plt.legend(loc = 'lower right', fontsize=10)
plt.xlim(0, 15)
plt.ylim(0,2)
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)


fig2.savefig(PLOT_PATH+'case2_cpa.png')
# ========================= figure 2 =========================================

plt.show()
