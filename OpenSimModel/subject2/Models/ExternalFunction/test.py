import casadi as ca
import numpy as np
import time

F_gen = ca.external('F', 'subject2_withMTP_weldRadius_scaled_FK_contactsAsForces_optimized.dll')
F_delete0 = ca.external('F', 'subject2_withMTP_weldRadius_scaled_FK_contactsAsForces_optimized_delete0.dll')
F_delete1 = ca.external('F', 'subject2_withMTP_weldRadius_scaled_FK_contactsAsForces_optimized_delete1.dll')

vec1 = np.zeros((99, 1))

start = time.time()
for i in range(int(1e5)):
    res_gen = (F_gen(vec1)).full()
elapsed_time_gen = (time.time() - start) 

start = time.time()
for i in range(int(1e5)):
    res_gen = (F_delete0(vec1)).full()
elapsed_time_delet0 = (time.time() - start)

start = time.time()
for i in range(int(1e5)):
    res_gen = (F_delete1(vec1)).full()
elapsed_time_delet1 = (time.time() - start)



print(str(elapsed_time_gen))
print(str(elapsed_time_delet0))
print(str(elapsed_time_delet1))

