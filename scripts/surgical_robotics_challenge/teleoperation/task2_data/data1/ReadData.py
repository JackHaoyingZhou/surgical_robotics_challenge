import numpy as np
from scipy.spatial.transform import Rotation as R
import pickle
from surgical_robotics_challenge.AI_project.pydmps.pydmps.cs import CanonicalSystem
from surgical_robotics_challenge.kinematics.psmFK import *
from surgical_robotics_challenge.kinematics.psmIK import *
from surgical_robotics_challenge.utils.utilities import convert_mat_to_frame

from PyKDL import Vector, Rotation, Frame, dot

file = 'data1.pickle'

with open(file,'rb') as fp:
    name_list, jp_list = pickle.load(fp)


index_init = 560

index_in = 6370

index_out = 7200


jp_value = []

jp_value.append(jp_list[index_init])

jp_new = []



jp_new.append(jp_list[index_init][0:6])
jp_new[0].append(0.3) # 0.0

for i in range(1000):
    jp_value.append(jp_list[index_init])
    jp_new.append(jp_list[index_init][0:6])
    if i < 900:
        jp_new[i+1].append(0.3) # 0.0
    else:
        jp_new[i + 1].append(0.0)

jp_value.append(jp_list[index_in])

jp_new.append(jp_list[index_in][0:6])
jp_new[1001].append(0.0)

for j in range(200):
    jp_value.append(jp_list[index_in])
    jp_new.append(jp_list[index_in][0:6])
    jp_new[j+1001].append(0.0)

for k in range(index_out - index_in):
    jp_value.append(jp_list[index_in+k])
    jp_new.append(jp_list[index_in+k][0:6])
    jp_new[k+1201].append(0.0)

name_new = name_list[0:len(jp_new)]

with open('data1_decouple.pickle','wb') as fp:
    pickle.dump([name_new, jp_new],fp)

# jp_value = jp_list[4800][0:6]
#
# jp_value.append(0.0)
#
# # jp_value.append(0.0)
#
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

