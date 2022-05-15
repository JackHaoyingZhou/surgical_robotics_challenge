import numpy as np
from scipy.spatial.transform import Rotation as R
import pickle
from surgical_robotics_challenge.Task2_project.pydmps.pydmps.cs import CanonicalSystem
from surgical_robotics_challenge.kinematics.psmFK import *
from surgical_robotics_challenge.kinematics.psmIK import *
from surgical_robotics_challenge.utils.utilities import convert_mat_to_frame

from PyKDL import Vector, Rotation, Frame, dot

file = 'data0.pickle'

with open(file,'rb') as fp:
    name_list, jp_list = pickle.load(fp)

index_in = 4800

index_out = 5600

jp_value = []

for i in range(index_out-index_in):
    jp_value.append(jp_list[index_in+i][0:6])
    jp_value[i].append(0.0)

# with open('data0_decouple.pickle','wb') as fp:
#     pickle.dump([name_new, jp_new],fp)

pos_list = []
ori_list = []

for j in range(len(jp_value)):
    T_f = compute_FK(jp_value[j], 7)
    # T_f_frame = convert_mat_to_frame(T_f)
    pos = np.transpose(np.array(T_f[0:3, 3]))[0]
    Rot = T_f[0:3, 0:3]
    r = R.from_matrix(Rot)
    ori = r.as_quat()
    pos_list.append(pos)
    ori_list.append(ori)

with open('data0_task.pickle','wb') as fp:
    pickle.dump([pos_list, ori_list],fp)

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

