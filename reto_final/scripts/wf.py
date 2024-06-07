import numpy as np
from pid import PIDController

def compute_wf_controller(closest_angle, distance_to_wall, target_distance, clk_cnt, dt):
        pid_angle = PIDController(Kp=2.4, Ki=0.0015, Kd=0.001)
        pid_distance = PIDController(Kp=0.9, Ki=0.001, Kd=0.001)


        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao = theta_ao - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_fw = -np.pi/2 + theta_ao if clk_cnt else np.pi/2 + theta_ao
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))
        angle_control = pid_angle.compute(theta_fw, dt)

        distance_error = distance_to_wall - target_distance
        distance_control = pid_distance.compute(distance_error, dt)

        w_fw = angle_control + distance_control  
        v_fw = 0.08

        return v_fw, w_fw


