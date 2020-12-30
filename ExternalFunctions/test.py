import casadi as ca
import numpy as np

PredSim = ca.external('F', 'PredSim.dll')  
PredSim_FD = ca.external('F', 'PredSim_FD.dll', dict(
                enable_fd=True, enable_forward=False, enable_reverse=False,
                enable_jacobian=False, fd_method='forward'))  

vec_in_1 = -np.ones((87, 1))
vec_in_2 = -np.ones((58, 1))
vec_in_3 = -np.ones((29, 1))
vec_out = (PredSim(vec_in_1)).full()  
vec_out_FD = (PredSim_FD(vec_in_2,vec_in_3)).full()   
assert np.alltrue(vec_out - vec_out_FD < 1e-8)
print(vec_out - vec_out_FD)

