import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

import numpy as np
import pickle

from surgical_robotics_challenge.Task2_project.pydmps.pydmps.dmp_discrete import DMPs_discrete

from surgical_robotics_challenge.Task2_project.quaternion_dmp import QuaternionDMP

file = 'data/data0/data0_task.pickle'

with open(file,'rb') as fp:
    pos_list, ori_list = pickle.load(fp)

file = 'data/data3/data3_task.pickle'

with open(file,'rb') as fp:
    pos_list_new, ori_list_new = pickle.load(fp)

t_list = []
t = 0
# t_list.append(t)

x_list = []
y_list = []
z_list = []
qx_list = []
qy_list = []
qz_list = []
qw_list = []

for i in range(len(pos_list)):
    x = pos_list[i][0]
    y = pos_list[i][1]
    z = pos_list[i][2]
    qx = ori_list[i][0]
    qy = ori_list[i][1]
    qz = ori_list[i][2]
    qw = ori_list[i][3]

    t_list.append(t)
    t = t + 0.005

    x_list.append(x)
    y_list.append(y)
    z_list.append(z)
    qx_list.append(qx)
    qy_list.append(qy)
    qz_list.append(qz)
    qw_list.append(qw)

pos_desire = []

pos_desire.append(x_list)
pos_desire.append(y_list)
pos_desire.append(z_list)

pos_desire = np.array(pos_desire)

# dmp = DMPs_discrete(dt=0.05, n_dmps=1, n_bfs=10, w=np.zeros((1, 10)))
# y_track, dy_track, ddy_track = dmp.rollout()

# dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=3, n_bfs=500, ay=np.ones(3)*25.0,by=np.ones(3)*4.0,dt= 1/500, ax = 1.0)

N_bf_pos = 200
N_bf_ori = 50
N_pts = 400

dmp_pos = DMPs_discrete(n_dmps=3, n_bfs=N_bf_pos, dt= 1/N_pts)

dmp_pos.imitate_path(pos_desire)

dmp_pos.y0 = pos_list_new[0]
dmp_pos.goal = pos_list_new[-1]

pos_gen, _, _ = dmp_pos.rollout()

ori_desire = np.array(ori_list)

dmp_ori = QuaternionDMP(N_bf=N_bf_ori, dt=1/N_pts)
q_des = dmp_ori.imitate(ori_desire)

dmp_ori.q0 = ori_list_new[0]
dmp_ori.dq_log0 = dmp_ori.logarithmic_map(dmp_ori.quaternion_error(ori_list[0], ori_list[1]))/dmp_ori.dt
dq1 = dmp_ori.logarithmic_map(dmp_ori.quaternion_error(ori_list[1], ori_list[2]))/dmp_ori.dt
ddq_list = []

dq_list = np.vstack((dmp_ori.dq_log0, dq1))
for d in range(3):
    ddq = np.gradient(dq_list[:, d]) / dmp_ori.dt
    ddq_list.append(ddq)
ddq_list = np.array(ddq_list)
dmp_ori.ddq_log0 = ddq_list[:,0]

dmp_ori.qT = ori_list_new[-1]


ori_gen, _, _ = dmp_ori.rollout()

N_new = len(pos_list_new)

t_1 = np.linspace(t_list[0], t_list[-1], len(pos_gen))
t_2 = np.linspace(t_list[0], t_list[-1], N_new)

# t_1 = []
# t1 = 0
# for i in range(len(pos_gen)):
#     t_1.append(t1)
#     t1 = t1 + 1/200
#
# t_2 = []
# t2 = 0
# for i in range(N_new):
#     t_2.append(t2)
#     t2 = t2 + 1/200


s0 = np.array(pos_list_new)

plt.figure(1)
plt.suptitle('position', fontsize=26)
plt.subplot(131)#
plt.plot(t_list,x_list,t_1,pos_gen[:,0])# f_pos, ax_pos = plt.subplots(3)
plt.plot(t_2, s0[:,0])
plt.legend(['model', 'dmp', 'original'], fontsize=16)# f_pos.suptitle('Positions')
plt.xlabel('time(s)', fontsize=20)# ax_pos[0].plot(t_list, x_list, 'r')
plt.ylabel('X', fontsize=20)# ax_pos[0].plot(t_list, x_list[], 'r')
plt.grid()# ax_pos[1].plot(t_list, y_list, 'r')
plt.subplot(132)# ax_pos[2].plot(t_list, z_list, 'r')
plt.plot(t_list,y_list,t_1,pos_gen[:,1])#
plt.plot(t_2, s0[:,1])
plt.legend(['model', 'dmp', 'original'], fontsize=16)# f_ori, ax_ori = plt.subplots(4)
plt.xlabel('time(s)', fontsize=20)# f_ori.suptitle('Orinetations')
plt.ylabel('Y', fontsize=20)# ax_ori[0].plot(t_list, qx_list, 'r')
plt.grid()# ax_ori[1].plot(t_list, qy_list, 'r')
plt.subplot(133)# ax_ori[2].plot(t_list, qz_list, 'r')
plt.plot(t_list,z_list,t_1,pos_gen[:,2])#  # ax_ori[3].plot(t_list, qw_list, 'r')
plt.plot(t_2, s0[:,2])
plt.legend(['model', 'dmp', 'original'], fontsize=16)
plt.xlabel('time(s)', fontsize=20)
plt.ylabel('Z', fontsize=20)
plt.grid()

plt.figure(3)
plt.suptitle('position', fontsize=26)
plt.subplot(131)#
plt.plot(t_list,x_list,t_1,pos_gen[:,0])# f_pos, ax_pos = plt.subplots(3)
plt.legend(['model', 'dmp'], fontsize=16)# f_pos.suptitle('Positions')
plt.xlabel('time(s)', fontsize=20)# ax_pos[0].plot(t_list, x_list, 'r')
plt.ylabel('X', fontsize=20)# ax_pos[0].plot(t_list, x_list[], 'r')
plt.grid()# ax_pos[1].plot(t_list, y_list, 'r')
plt.subplot(132)# ax_pos[2].plot(t_list, z_list, 'r')
plt.plot(t_list,y_list,t_1,pos_gen[:,1])#
plt.legend(['model', 'dmp'], fontsize=16)# f_ori, ax_ori = plt.subplots(4)
plt.xlabel('time(s)', fontsize=20)# f_ori.suptitle('Orinetations')
plt.ylabel('Y', fontsize=20)# ax_ori[0].plot(t_list, qx_list, 'r')
plt.grid()# ax_ori[1].plot(t_list, qy_list, 'r')
plt.subplot(133)# ax_ori[2].plot(t_list, qz_list, 'r')
plt.plot(t_list,z_list,t_1,pos_gen[:,2])#  # ax_ori[3].plot(t_list, qw_list, 'r')
plt.legend(['model', 'dmp'], fontsize=16)
plt.xlabel('time(s)', fontsize=20)
plt.ylabel('Z', fontsize=20)
plt.grid()

s = np.array(ori_list_new)



plt.figure(2)
plt.suptitle('orientation', fontsize=26)
plt.subplot(221)#
plt.plot(t_list,qx_list,t_1,ori_gen[:,0])# f_pos, ax_pos = plt.subplots(3)
plt.plot(t_2, s[:,0])
plt.legend(['model', 'dmp', 'original'], fontsize=16)# f_pos.suptitle('Positions')
plt.xlabel('time(s)', fontsize=20)# ax_pos[0].plot(t_list, x_list, 'r')
plt.ylabel('Qx', fontsize=20)# ax_pos[0].plot(t_list, x_list[], 'r')
plt.grid()# ax_pos[1].plot(t_list, y_list, 'r')
plt.subplot(222)# ax_pos[2].plot(t_list, z_list, 'r')
plt.plot(t_list,qy_list,t_1,ori_gen[:,1])#
plt.plot(t_2, s[:,1])
plt.legend(['model', 'dmp', 'original'], fontsize=16)# f_ori, ax_ori = plt.subplots(4)
plt.xlabel('time(s)', fontsize=20)# f_ori.suptitle('Orinetations')
plt.ylabel('Qy', fontsize=20)# ax_ori[0].plot(t_list, qx_list, 'r')
plt.grid()# ax_ori[1].plot(t_list, qy_list, 'r')
plt.subplot(223)# ax_ori[2].plot(t_list, qz_list, 'r')
plt.plot(t_list,qz_list,t_1,ori_gen[:,2])#  # ax_ori[3].plot(t_list, qw_list, 'r')
plt.plot(t_2, s[:,2])
plt.legend(['model', 'dmp', 'original'], fontsize=16)
plt.xlabel('time(s)', fontsize=20)
plt.ylabel('Qz', fontsize=20)
plt.grid()
plt.subplot(224)# ax_ori[2].plot(t_list, qz_list, 'r')
plt.plot(t_list,qw_list,t_1,ori_gen[:,3])#  # ax_ori[3].plot(t_list, qw_list, 'r')
plt.plot(t_2, s[:,3])
plt.legend(['model', 'dmp', 'original'], fontsize=16)
plt.xlabel('time(s)', fontsize=20)
plt.ylabel('Qw', fontsize=20)
plt.grid()

plt.show()
#
# with open('data/data2/data2_train_modify.pickle','wb') as fp:
#     pickle.dump([pos_gen, ori_gen], fp)