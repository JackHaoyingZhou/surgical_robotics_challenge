#!/usr/bin/env python

# this script is for data loading

# //==============================================================================
# /*
#     \author    <hzhou6@wpi.edu>
#     \author    Haoying Zhou
#     \version   1.0
# */
# //==============================================================================


### make sure the path is correct
import os
import sys
# folder_name = "/teleoperation/task_data/1"
# dynamic_path = os.path.abspath(__file__+"/../../"+folder_name)
# print(dynamic_path)
# sys.path.append(dynamic_path)

from surgical_robotics_challenge.utils.joint_pos_recorder import JointPosLoader
import pickle



m,l = JointPosLoader.load_by_prefix('JP#2022-04-29')

jp_values = []

for i in range(len(m)):
    for j in range(len(m[0])):
        jp_values.append(m[i][j]['pos'])

name_list = []
jp_list = []

for i in range(len(jp_values)):
    name_list.append(str(jp_values[i][0]))
    joint_values = [jp_values[i][1][0],jp_values[i][1][1],jp_values[i][1][2],
                    jp_values[i][1][3],jp_values[i][1][4],jp_values[i][1][5],
                    jp_values[i][2]]
    jp_list.append(joint_values)

#
with open('data3.pickle','wb') as fp:
    pickle.dump([name_list, jp_list],fp)

with open('data3.pickle','rb') as fp:
    itemlist_1, itemlist_2 = pickle.load(fp)
