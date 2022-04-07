# from psmIK import *
from ambf_client import Client
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
import time
import rospy
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Control PSM3', default=False) ## if use PSM3, change False to True

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    if parsed_args.run_psm_one in ['True', 'true', '1']:
        parsed_args.run_psm_one = True
    elif parsed_args.run_psm_one in ['False', 'false', '0']:
        parsed_args.run_psm_one = False

    if parsed_args.run_psm_two in ['True', 'true', '1']:
        parsed_args.run_psm_two = True
    elif parsed_args.run_psm_two in ['False', 'false', '0']:
        parsed_args.run_psm_two = False
    if parsed_args.run_psm_three in ['True', 'true', '1']:
        parsed_args.run_psm_three = True
    elif parsed_args.run_psm_three in ['False', 'false', '0']:
        parsed_args.run_psm_three = False

    c = Client()
    c.connect()

    cam = ECM(c, 'CameraFrame')

    time.sleep(0.5)

    controllers = []
    psm_arms = []

    if parsed_args.run_psm_one is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm1'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            T_psmtip_c = Frame(Rotation.RPY(3.14, 0.0, -1.57079), Vector(-0.2, 0.0, -1.0))
            T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm.set_home_pose(T_psmtip_b)
            psm_arms.append(psm)

    if parsed_args.run_psm_two is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm2'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            T_psmtip_c = Frame(Rotation.RPY(3.14, 0.0, -1.57079), Vector(0.2, 0.0, -1.0))
            T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm.set_home_pose(T_psmtip_b)
            psm_arms.append(psm)

    if parsed_args.run_psm_three is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm3'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            psm_arms.append(psm)


    #### set ECM pose
    cam.servo_jp([0, 0, 0, 0])

    #### set PSM1 pose
    psm_arms[0].servo_jp([0.30279368532551626, -0.218422932338383, 1.616375329074801, 1.1810708577823035, -0.13133471052627893, 0.9791169287496])
    ##################### [+/- 0.1]            [+/- 0.1]            [+/- 0.05]        [+/- 1.0]           [+/- 1.0]             [+/- 1.0]
    psm_arms[0].set_jaw_angle(0.3)
    ################# jaw angle = 0 is closed for the grippper

    #### set PSM2 pose
    psm_arms[1].servo_jp([-0.2949609515342975, -0.2185843809601709, 1.5540725484395173, 1.6359442582525876, 0.26205555484769827, 0.6602696229205932])
    ##################### [+/- 0.1]            [+/- 0.1]            [+/- 0.05]        [+/- 1.0]           [+/- 1.0]             [+/- 1.0]
    psm_arms[1].set_jaw_angle(0.3)


