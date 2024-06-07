import numpy as np

def compute_gtg_control(xg, yg, x, y, theta): 
        kvmax = 0.1 #linear speed maximum gain  
        kwmax = 0.4 #angular angular speed maximum gain 

        av = 0.5 #Constant to adjust the exponential's growth rate   
        aw = 1.0 #Constant to adjust the exponential's growth rate 

        ed = np.sqrt((xg-x)**2+(yg-y)**2) 

        #Compute angle to the target position 

        theta_target = np.arctan2(yg-y,xg-x) 
        e_theta = theta_target - theta 

        #limit e_theta from -pi to pi 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 
        #Compute the robot's angular speed 
        kw = kwmax*(1-np.exp(-aw*e_theta**2))/abs(e_theta) #Constant to change the speed  
        w = kw*e_theta 

        if abs(e_theta) > np.pi/8: 
            v = 0 #linear speed  

        else: 
            kv = kvmax*(1-np.exp(-av*ed**2))/abs(ed) #Constant to change the speed  
            v = 0.05  #linear speed  

        return v, w