import numpy as np

def is_point_near_segment(x1, y1, x2, y2, x3, y3, epsilon):
        a, b, c = calculate_line_equation(x2, y2, x3, y3)
        numerator = np.abs(a*x1 + b*y1 + c)
        denominator =  np.sqrt(a*a + b*b)
        distance = numerator / denominator
        
        if distance <= epsilon:
            return True, distance
        return False, distance

def calculate_line_equation(x1, y1, x2, y2):
        a = y2 - y1
        b = x1 - x2
        c = x2 * y1 - x1 * y2

        return a, b, c

def quit_wf_bug_two(theta_gtg, theta_ao, xg, yg, x, y, x_init, y_init):
        n_segment, distance = is_point_near_segment(x, y, x_init, y_init, xg, yg, 0.12)
        print("Distance: ",distance)
        print("Clear? : ", (np.abs(theta_ao - theta_gtg)) < np.pi/2)
        print()

        if (n_segment) and (np.abs(theta_ao - theta_gtg) < np.pi/2) and (np.abs(theta_gtg) < np.pi/2):
            return True
        else:
            return False
        
def quit_wf_bug_2(x_init , y_init , xg , yg , x_pos , y_pos):
    tolerance = 0.06
    if x_init == xg: xg += 0.0001
             
    m = (yg - y_init)/(xg - x_init)
    b = y_init - m * x_init
    y_line = m * x_pos + b
    return y_line - tolerance < y_pos < y_line + tolerance

def crash_state(cl_distance , cl_angle , follow_distance):
      distance = cl_distance <= follow_distance
      angle = cl_angle > -np.pi/4 and cl_angle < np.pi/4
      return distance and angle
      
        
def quit_wf_bug_two_t(xg, yg, x, y, x_tmp, y_tmp):
      d_tmp = np.sqrt((xg - x_tmp)**2 + (yg - y_tmp)**2) - 0.07
      d_r = np.sqrt((xg - x)**2 + (yg - y)**2)
      print("quit_wf_bug_two_t")
      print (d_r, d_tmp)
      return d_tmp > d_r
      

def quit_wf_bug_zero(theta_gtg, theta_ao, d_t, d_t1):
        print("D(h1): ",d_t1)
        print("D: ",d_t)
        print("Clear? : ", np.abs(theta_ao - theta_gtg))
        print()
        if (d_t < d_t1) and (np.abs(theta_ao - theta_gtg) < np.pi/2):
            return True
        else:
            return False
    
def clockwise_counter(xg, yg, x, y, theta, closest_angle):
        theta_target=np.arctan2(yg-y,xg-x) 
        e_theta=theta_target-theta
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 

        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao = theta_ao - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_fw = -np.pi/2 + theta_ao
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))
    
        if np.abs(theta_fw - e_theta) <= np.pi/2:
            return 1
        else:
            return 0

def compute_angles(xg, yg, x, y, theta, closest_angle):
        theta_target=np.arctan2(yg-y,xg-x) 
        e_theta=theta_target-theta
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 

        theta_ao = closest_angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        theta_ao = theta_ao - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        return e_theta, theta_ao