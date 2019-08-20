3dpredictsim
============

This repository contains code and data to generate three-dimensional muscle-driven predictive simulations of human gaits, and to reproduce all results presented in: Falisse A, Serrancoli G, Dembia C, Gillis J, Jonkers J, De Groote F (2019), "Rapid predictive simulations with complex musculoskeletal models suggest that diverse healthy and pathological human gaits can emerge from similar control strategies". Journal of the Royal Society Interface.

Thanks for citing our work in any derived publication. Feel free to reach us for any questions: antoine.falisse@kuleuven.be | antoinefalisse@gmail.com | friedl.degroote@kuleuven.be. This code has been developed on Windows using MATLAB2017b. There is no guarantee that it runs smooth on other platforms. Please let us know if you run into troubles.

3dpredictsim contains different folders with data and code needed to perform the predictive and tracking simulations as well as to reproduce all figures from the study. The best way to get started is to run OCP/PredSim_all.m and to
explore the code from there (make sure you install CasADi beforehand:
[CasADi website](https://web.casadi.org/)

Here, we provide a brief description of the different scripts and folders. 
 
OCP (Optimal Control Problems)
==============================

Folder with scripts performing the predictive and tracking simulations.
    1. PredSim_all.m
        - Script that formulates the predictive simulations (except those with the prosthesis).
    2. PredSim_prosthesis.m
        - Script that formulates the predictive simulations with the prosthesis.
    3. TrackSim.m
        - Script that formulates the tracking simulation.
  
MuscleModel
=========== 

Folder with scripts and data describing muscle activation dynamics, muscle contraction dynamics, and arm activation dynamics.
             This folder also contains helper functions for use when formulating the optimal control problems.
    1. Scripts:
        1. ArmActivationDynamics.m
            - Function that describes the dynamics of the arms.
        2. computeExcitationRaasch.m
            - Function that computes muscle excitations from time derivative of muscle activations.
        3. FiberLength_TendonForce_tendon.m
            - Function that computes fiber lengths from muscle-tendon forces.
        4. FiberVelocity_TendonForce_tendon.m
            - Function that computes fiber velocities from muscle-tendon forces.
        5. ForceEquilibrium_FtildeState_all.m
            - Function that derives the Hill-equilibrium.
        6. ForceEquilibrium_FtildeState_all_tendon.m
            - Function that derives the Hill-equilibrium with tendon stiffness as parameter.
        7. MomentArmIndices.m        
            - Helper function that returns indices for use with the moment arms.
        8. MuscleIndices.m
            - Helper function that returns indices for use with the muscles. 
    2. Data:
        1. Faparam.mat
            - Parameters of the muscle active force-length relationship.
        2. Fpparam.mat
            - Parameters of the muscle passive force-length relationship.
        3. Fvparam.mat
            - Parameters of the muscle force-velocity relationship.
        4. MTparameters_subject1.mat
            - Muscle-tendon parameters: 
                Row 1: maximal isometric forces; Row 2: optimal fiber lengths; Row 3: tendon slack lengths; 
                Row 4: optimal pennation angles; Row 5: maximal contraction velocities. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
Polynomials: folder with scripts and data describing muscle-tendon lengths, velocities, and moment arms using polynomial expressions
             of joint positions and velocities.
    Scripts:
        - Main_polynomials_subject1.m
            - Script that sets the process of calibrating the polynomial coefficients.
        - PolynomialFit.m
            - Function that calibrates the polynomial coefficients.
        - n_art_mat_3.m and n_art_mat_3_cas_SX.m
            - Functions that provide the polynomials based on the number of degrees of freedom and the polynomial order.
            - n_art_mat_3_cas_SX.m is derived from n_art_mat_3.m and adjusted for use with CasADi.
    Data:
        - muscle_spanning_joint_INFO_subject1.mat
            - Helper matrix indicating which muscle actuates which degree of freedom.
        - MuscleInfo_subject1.mat
            - Structure with polynomial coefficients.
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
MetabolicEnergy: folder with scripts describing the metabolic energy models.
    - getMetabolicEnergySmooth2003all.m
        - Function that describes the metabolic energy model from Umberger et al. (2003).
    - getMetabolicEnergySmooth2004all.m
        - Function that describes the metabolic energy model from Bhargava et al. (2004).
    - getMetabolicEnergySmooth2010all.m
        - Function that describes the metabolic energy model from Umberger (2010).
    - getMetabolicEnergySmooth2016all.m
        - Function that describes the metabolic energy model from Uchida et al. (2016).
    - getMetabolicEnergySmooth2010all_hl.m
        - Function that describes the metabolic energy model from Umberger (2010) treating muscle lengthening heat rate as Umberger et al. (2003).
    - getMetabolicEnergySmooth2010all_neg.m
        - Function that describes the metabolic energy model from Umberger (2010) treating negative mechanical work as Umberger et al. (2003).
    - getPctSTSmooth.m
        - Function that describes the orderly recruitment model.
    - getSlowTwitchRatios.m
        - Helper function that returns the percentage of slow twitch fibers in the muscles.
    - getSpecificTensions.m
        - Helper function that returns the specific tension of the muscles.
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
PassiveMoments: folder with the script describing the passive joint torques.
    - PassiveMomentsData.m
        - Script that describes the passive joint torques.
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
CasADiFunctions: folder with scripts implementing several CasADi-based functions.
    - CasADiFunctions_all.m
        - Script for use in PredSim_all.m.
    - CasADiFunctions_prosthesis.m
        - Script for use in PredSim_prosthesis.m.
    - CasADiFunctions_tracking.m
        - Script for use in TrackSim.m.
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
Contact: folder with the script describing the foot-ground contact model (for use in tracking simulation).
    - HCContactModel.m
        - Function that describes the Hunt-Crossley foot-ground contact model.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
CollocationScheme: folder with the script describing the collocation scheme.
    - CollocationScheme.m
        - Script that describes the collocation scheme.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
Bounds: folder with the scripts setting the bounds of the optimal control problems.
    - getBounds_all.m
        - Script for use in PredSim_all.m.
    - getBounds_prosthesis.m
        - Script for use in PredSim_prosthesis.m.
    - getBounds_tracking.m
        - Script for use in TrackSim.m.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
IG (Initial Guesses): folder with the scripts setting the initial guesses of the optimal control problems.
    - getGuess_QR.m
        - Script that describes the quasi-random initial guess.
    - getGuess_QR_prosthesis.m
        - Script that describes the quasi-random initial guess for the prosthesis case.
    - getGuess_DI.m
        - Script that describes the data-informed initial guess.
    - getGuess_DI_prosthesis.m
        - Script that describes the data-informed initial guess for the prosthesis case.
    - getGuess_DI_t.m
        - Script that describes the data-informed initial guess with the final time based on provided data.
    - getGuess_DI_tracking.m
        - Script that describes the data-informed initial guess for the tracking simulation.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
Plots: folder with scripts plotting initial guesses versus bounds.
    - plot_BoundsVSInitialGuess_all.m
        - Script for use in PredSim_all.m.
    - plot_BoundsVSInitialGuess_prosthesis.m
        - Script for use in PredSim_prosthesis.m.
    - plot_BoundsVSInitialGuess_tracking.m
        - Script for use in TrackSim.m.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
VariousFunctions: folder with scripts with various purposes.
    - barwitherr.m
        - Function that generates bar plots with error bars.
    - getGRF.m
        - Function that extracts ground reaction forces.
    - getID.m
        - Function that extracts inverse dynamics results.
    - getIK.m
        - Function that extracts inverse kinematics results.
    - readDiary.m
        - Function that reads diaries from IPOPT.
    - write_motionFile.m
        - Function that writes motion files for OpenSim.
    - SplineEval_ppuval.m
        - Script that evaluates splines.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
ResultsAnalysis: folder with scripts reproducing all figures of the study.
    - Fig1
        - Script that reproduces Fig 1.
    - Fig2
        - Script that reproduces Fig 2.
    - Fig3A
        - Script that reproduces Fig 3A.
    - Fig3B
        - Script that reproduces Fig 3B.
    - Fig4
        - Script that reproduces Fig 4.
    - FigS1
        - Script that reproduces Fig S1.
    - FigS2
        - Script that reproduces Fig S2.
    - FigS3
        - Script that reproduces Fig S3.
    - FigS4
        - Script that reproduces Fig S4.
    - FigS5
        - Script that reproduces Fig S5.
    - FigS6
        - Script that reproduces Fig S6.
    - getCPU_all.m
        - Script that extracts the CPU times from the simulation results.
    - observeResults_PredSim.m
        - Scripts that plot results not presented in the paper.
    - predSim_data_all.m
        - Script that loads the simulation results.
    - predSim_settings_all.m
        - Script that loads the settings of the optimal control problems.
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
OpenSimModel\subject1: folder with data from subject1.
    - GRF
        - GRF_gait_1.mat
            - Ground reaction forces (for use in tracking simulation). 
    - ID
        - ID_gait_1.mat
            - Joint torques (for use in tracking simulation).
    - IK
        - IK_average_running_HGC.mat
            - Joint positions from running, averaged across experimental trials, for half a gait cycle.
            - For use with data-informed (running) initial guess.
        - IK_average_walking_FGC.mat
            - Joint positions from walking, averaged across experimental trials, for a full a gait cycle.
            - For use with data-informed (walking) initial guess: prosthesis case.
        - IK_average_walking_HGC.mat
            - Joint positions from walking, averaged across experimental trials, for half a gait cycle.
            - For use with data-informed (walking) initial guess.
        - IK_gait_1.mat
            - Joint positions (for use in tracking simulation).
    - subject1.osim
        - OpenSim model used in this study.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
ExperimentalData: folder with experimental data.
    - ExperimentalData.mat
        - Structure with experimental data: more information in the readme of that folder. 
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
Results: folder with simulation results.
    - PredSim_all/Results_all.mat
        - Structure with all results from the predictive simulations except those with the prosthesis.
    - PredSim_prosthesis/Results_prosthesis.mat
        - Structure with results from the predictive simulations with the prosthesis.
    - TrackSim/Results_tracking.mat
        - Structure with results from the tracking simulations.
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
ExternalFunctions: folder with external functions, more information in the readme of that folder. 
