import time
from tkinter import *
from surgical_robotics_challenge.simulation_manager import SimulationManager
from PyKDL import Vector, Rotation, Frame
from surgical_robotics_challenge.utils.utilities import cartesian_interpolate_step
import numpy as np
import time
from surgical_robotics_challenge.utils import coordinate_frames


class NeedleInitialization:
    def __init__(self, simulation_manager):
        self.T_needle_psmtip = coordinate_frames.Needle.T_center_psmtip
        self.T_needle_psmtip_far = self.T_needle_psmtip * Frame(Rotation.RPY(0., 0., 0.), Vector(0., 0., 0.))

        self.needle = simulation_manager.get_obj_handle('Needle')
        time.sleep(1.0)
        self._release = False
        self._reached = False

    def get_tip_to_needle_offset(self):
        return self.T_needle_psmtip

    def move_to_psm1(self, psm_tip):
        T_needle_psmtip_offset = self.T_needle_psmtip_far * Frame(Rotation.RPY(-np.pi/2 , 0., np.pi), Vector(-0.10727960616350174, -0.07585766911506653, -0.013998392969369888))
        print('Moving Needle to PSM 1 Tip')
        self._release = False
        if psm_tip is None:
            print('Not a valid link, returning')
            return
        T_nINw = self.needle.get_pose()
        T_tINw = psm_tip.get_pose()
        # First reach the farther point
        self._reached = False
        done = False
        while not done:
            T_nINw_cmd = T_tINw * T_needle_psmtip_offset
            T_delta, done = cartesian_interpolate_step(T_nINw, T_nINw_cmd, 0.01, 0.005)
            r_delta = T_delta.M.GetRPY()
            # print(error_max)
            T_cmd = Frame()
            T_cmd.p = T_nINw.p + T_delta.p
            T_cmd.M = T_nINw.M * Rotation.RPY(r_delta[0], r_delta[1], r_delta[2])
            T_nINw = T_cmd
            self.needle.set_pose(T_cmd)
            time.sleep(0.01)

        time.sleep(0.5)
        done = False
        T_nINw = self.needle.get_pose()
        T_tINw = psm_tip.get_pose()
        while not done:
            T_nINw_cmd = T_tINw * self.T_needle_psmtip
            T_delta, done = cartesian_interpolate_step(T_nINw, T_nINw_cmd, 0.01, 0.005)
            r_delta = T_delta.M.GetRPY()
            T_cmd = Frame()
            T_cmd.p = T_nINw.p + T_delta.p
            T_cmd.M = T_nINw.M * Rotation.RPY(r_delta[0], r_delta[1], r_delta[2])
            T_nINw = T_cmd
            self.needle.set_pose(T_cmd)
            time.sleep(0.01)

        self._reached = True

    def move_to_psm2(self, psm_tip):
        T_needle_psmtip_offset = self.T_needle_psmtip_far * Frame(Rotation.RPY(np.pi/2., 0., 0.), Vector(0., 0., -0.010))
        print('Moving Needle to PSM 2 Tip')
        self._release = False
        if psm_tip is None:
            print('Not a valid link, returning')
            return
        T_nINw = self.needle.get_pose()
        T_tINw = psm_tip.get_pose()
        # First reach the farther point
        self._reached = False
        done = False
        while not done:
            T_nINw_cmd = T_tINw * T_needle_psmtip_offset
            T_delta, done = cartesian_interpolate_step(T_nINw, T_nINw_cmd, 0.01, 0.005)
            r_delta = T_delta.M.GetRPY()
            # print(error_max)
            T_cmd = Frame()
            T_cmd.p = T_nINw.p + T_delta.p
            T_cmd.M = T_nINw.M * Rotation.RPY(r_delta[0], r_delta[1], r_delta[2])
            T_nINw = T_cmd
            self.needle.set_pose(T_cmd)
            time.sleep(0.01)

        time.sleep(0.5)
        done = False
        T_nINw = self.needle.get_pose()
        T_tINw = psm_tip.get_pose()
        while not done:
            T_nINw_cmd = T_tINw * self.T_needle_psmtip
            T_delta, done = cartesian_interpolate_step(T_nINw, T_nINw_cmd, 0.01, 0.005)
            r_delta = T_delta.M.GetRPY()
            T_cmd = Frame()
            T_cmd.p = T_nINw.p + T_delta.p
            T_cmd.M = T_nINw.M * Rotation.RPY(r_delta[0], r_delta[1], r_delta[2])
            T_nINw = T_cmd
            self.needle.set_pose(T_cmd)
            time.sleep(0.01)

        self._reached = True

    def release(self):
        print('Releasing Needle')
        self._release = True
        self.needle.set_force(Vector(0, 0, 0))
        self.needle.set_torque(Vector(0, 0, 0))

    def has_reached(self):
        return self._reached

simulation_manager = SimulationManager('attach_needle')
link1 = simulation_manager.get_obj_handle('psm1' + '/toolyawlink')
link2 = simulation_manager.get_obj_handle('psm2' + '/toolyawlink')
needle = NeedleInitialization(simulation_manager)

def psm1_btn_cb():
    needle.move_to_psm1(link1)
    time.sleep(5.0)
    needle.release()
    time.sleep(0.5)


def psm2_btn_cb():
    needle.move_to_psm2(link2)
    time.sleep(5.0)
    needle.release()
    time.sleep(0.5)


if __name__ == "__main__":
    tk = Tk()
    tk.title("Attache Needle")
    tk.geometry("250x250")
    link1_button = Button(tk, text="PSM 1", command=psm1_btn_cb,
                          height=3, width=50, bg="red")
    link2_button = Button(tk, text="PSM 2", command=psm2_btn_cb,
                          height=3, width=50, bg="blue")

    link1_button.pack()
    link2_button.pack()

    tk.mainloop()

