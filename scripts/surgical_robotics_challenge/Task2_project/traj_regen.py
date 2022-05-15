import numpy as np
from scipy.spatial.transform import Rotation as R
import pickle
from surgical_robotics_challenge.kinematics.psmIK import *
from surgical_robotics_challenge.utils.utilities import convert_mat_to_frame

# ##### for data0
# file_task = 'data/data0/data0_train.pickle'
# with open(file_task,'rb') as fp:
#     pos_list, ori_list = pickle.load(fp)
#
# file_joint = 'data/data0/data0.pickle'
# with open(file_joint,'rb') as fp:
#     name_list, jp_list = pickle.load(fp)
#
# index_init = 800
# index_in = 4800
# index_out = 5600

# ##### for data1
# file_task = 'data/data1/data1_train.pickle'
#
# with open(file_task,'rb') as fp:
#     pos_list, ori_list = pickle.load(fp)
#
# file_joint = 'data/data1/data1.pickle'
#
# with open(file_joint,'rb') as fp:
#     name_list, jp_list = pickle.load(fp)
#
# T_f = np.zeros((4,4))
#
# index_init = 560
#
# index_in = 6370
#
# index_out = 7200

# ##### for data2
# file_task = 'data/data2/data2_train.pickle'
#
# with open(file_task,'rb') as fp:
#     pos_list, ori_list = pickle.load(fp)
#
# file_joint = 'data/data2/data2.pickle'
#
# with open(file_joint,'rb') as fp:
#     name_list, jp_list = pickle.load(fp)
#
# T_f = np.zeros((4,4))
#
# index_init = 510
#
# index_in = 4800
#
# index_out = 5800

# ##### for data3
# file_task = 'data/data3/data3_train.pickle'
#
# with open(file_task,'rb') as fp:
#     pos_list, ori_list = pickle.load(fp)
#
# file_joint = 'data/data3/data3.pickle'
#
# with open(file_joint,'rb') as fp:
#     name_list, jp_list = pickle.load(fp)
#
# T_f = np.zeros((4,4))
#
# index_init = 600
#
# index_in = 4180
#
# index_out = 5200

# ##### for data4
# file_task = 'data/data4/data4_train.pickle'
#
# with open(file_task,'rb') as fp:
#     pos_list, ori_list = pickle.load(fp)
#
# file_joint = 'data/data4/data4.pickle'
#
# with open(file_joint,'rb') as fp:
#     name_list, jp_list = pickle.load(fp)
#
# T_f = np.zeros((4,4))
#
# index_init = 700
#
# index_in = 4660
#
# index_out = 6000

##### for data5
file_task = 'data/data5/data5_train.pickle'

with open(file_task,'rb') as fp:
    pos_list, ori_list = pickle.load(fp)

file_joint = 'data/data5/data5.pickle'

with open(file_joint,'rb') as fp:
    name_list, jp_list = pickle.load(fp)

T_f = np.zeros((4,4))

index_init = 600

index_in = 4250

index_out = 5400



T_f = np.zeros((4,4))
N_pts = len(pos_list)


jp_value = []

jp_value.append(jp_list[index_init])
#
# jp_new = []


# for i in range(1000):
#     jp_value.append(jp_list[index_init][0:6])
#     if i < 900:
#         jp_value[i+1].append(0.3) # 0.0
#     else:
#         jp_value[i + 1].append(0.0)

#### for data5 only
for i in range(1000):
    jp_value.append(jp_list[index_init][0:6])
    if i < 700:
        jp_value[i+1].append(0.3) # 0.0
    else:
        jp_value[i + 1].append(0.0)

# #### for data 0-2
# for j in range(400):
#     jp_value.append(jp_list[index_in][0:6])
#     jp_value[j+1001].append(0.0)

# ##### for data 3-4
# for j in range(800):
#     jp_value.append(jp_list[index_in][0:6])
#     jp_value[j+1001].append(0.0)

##### for data5
for j in range(2500):
    jp_value.append(jp_list[index_in][0:6])
    jp_value[j+1001].append(0.0)

for k in range(N_pts):
    r = R.from_quat(ori_list[k])
    T_f[0:3, 0:3] = r.as_matrix()
    T_f[0:3, 3] = pos_list[k]
    T_f_frame = convert_mat_to_frame(T_f)
    ik_sol = compute_IK(T_f_frame)
    ik_sol = enforce_limits(ik_sol)
    ik_sol.append(jp_list[index_in+k-1][6])
    jp_value.append(ik_sol)


name_new = name_list[0:len(jp_value)]

# with open('data/data0/data0_regen.pickle','wb') as fp:
#     pickle.dump([name_new, jp_value], fp)

# with open('data/data1/data1_regen.pickle','wb') as fp:
#     pickle.dump([name_new, jp_value], fp)

# with open('data/data2/data2_regen.pickle','wb') as fp:
#     pickle.dump([name_new, jp_value], fp)

# with open('data/data3/data3_regen.pickle','wb') as fp:
#     pickle.dump([name_new, jp_value], fp)

# with open('data/data5/data5_regen.pickle','wb') as fp:
#     pickle.dump([name_new, jp_value], fp)





# T_f = compute_FK(jp_value, 7)
# #
# T_f_frame = convert_mat_to_frame(T_f)
# # ik_sol = compute_IK(T_f_frame)
# # ik_sol = enforce_limits(ik_sol)
# #
# # ik_sol.append(jp_list[1][-1])
# #
# Pos = T_f[0:3, 3]
# Rot = T_f[0:3, 0:3]
#
# r = R.from_matrix(Rot)
#
# a = r.as_quat()   # [x,y,z,w]
