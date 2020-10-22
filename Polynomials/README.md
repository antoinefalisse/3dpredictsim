Polynomials
===========
Folder with scripts and data describing muscle-tendon lengths, velocities, and moment arms using polynomial expressions of joint positions and velocities.

1. Scripts:
    1. polynomialCoefficientEstimation_main.m
        - Script that sets the process of calibrating the polynomial coefficients.
    2. polynomialCoefficientEstimation.m
        - Function that calibrates the polynomial coefficients.
    3. n_art_mat_3.m and n_art_mat_3_cas_SX.m
        - Functions that provide the polynomials based on the number of degrees of freedom and the polynomial order.
        - n_art_mat_3_cas_SX.m is derived from n_art_mat_3.m and adjusted for use with CasADi.
2. Data:
    1. muscle_spanning_joint_INFO_subject1.mat
        - Helper matrix indicating which muscle actuates which degree of freedom.
    2. MuscleInfo_subject1.mat
        - Structure with polynomial coefficients.        
