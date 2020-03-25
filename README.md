3dpredictsim
============

This repository contains code and data to generate three-dimensional muscle-driven predictive simulations of human gait as described in: Falisse A, Serrancoli G, Dembia C, Gillis J, Jonkers J, De Groote F. 2019 Rapid predictive simulations with complex musculoskeletal models suggest that diverse healthy and pathological human gaits can emerge from similar control strategies. Journal of the Royal Society Interface 16: 20190402. http://dx.doi.org/10.1098/rsif.2019.0402

Thanks for citing our work in any derived publication. Feel free to reach us for any questions: antoine.falisse@kuleuven.be | antoinefalisse@gmail.com | friedl.degroote@kuleuven.be. This code has been developed on Windows using MATLAB2017b. There is no guarantee that it runs smooth on other platforms. Please let us know if you run into troubles.

3dpredictsim contains different folders with data and code needed to reproduce all results and figures from the study. The best way to get started is to run OCP/PredSim_all.m and to explore the code from there (make sure you install CasADi beforehand: [see CasADi website](https://web.casadi.org/))

Here, we provide a brief description of the different scripts and folders. 
 
### OCP (Optimal Control Problems)

Folder with scripts performing the predictive and tracking simulations.

1. PredSim_all.m
    - Script that formulates the predictive simulations (except those with the prosthesis).
2. PredSim_prosthesis.m
    - Script that formulates the predictive simulations with the prosthesis.
3. TrackSim.m
    - Script that formulates the tracking simulation.
  
### MuscleModel

Folder with scripts and data describing muscle activation dynamics, muscle contraction dynamics, and arm activation dynamics. This folder also contains helper functions for use when formulating the optimal control problems.

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
            1. Row 1: maximal isometric forces
            2. Row 2: optimal fiber lengths
            3. Row 3: tendon slack lengths
            4. Row 4: optimal pennation angles
            5. Row 5: maximal contraction velocities

### Polynomials

Folder with scripts and data describing muscle-tendon lengths, velocities, and moment arms using polynomial expressions of joint positions and velocities.

1. Scripts:
    1. Main_polynomials.m
        - Script that sets the process of calibrating the polynomial coefficients.
    2. PolynomialFit.m
        - Function that calibrates the polynomial coefficients.
    3. n_art_mat_3.m and n_art_mat_3_cas_SX.m
        - Functions that provide the polynomials based on the number of degrees of freedom and the polynomial order.
        - n_art_mat_3_cas_SX.m is derived from n_art_mat_3.m and adjusted for use with CasADi.
2. Data:
    1. muscle_spanning_joint_INFO_subject1.mat
        - Helper matrix indicating which muscle actuates which degree of freedom.
    2. MuscleInfo_subject1.mat
        - Structure with polynomial coefficients.
        
### MetabolicEnergy

Folder with scripts describing the metabolic energy models.

1. getMetabolicEnergySmooth2003all.m
    - Function that describes the metabolic energy model from Umberger et al. (2003).
2. getMetabolicEnergySmooth2004all.m
    - Function that describes the metabolic energy model from Bhargava et al. (2004).
3. getMetabolicEnergySmooth2010all.m
    - Function that describes the metabolic energy model from Umberger (2010).
4. getMetabolicEnergySmooth2016all.m
    - Function that describes the metabolic energy model from Uchida et al. (2016).
5. getMetabolicEnergySmooth2010all_hl.m
    - Function that describes the metabolic energy model from Umberger (2010) treating muscle lengthening heat rate as Umberger et al. (2003).
6. getMetabolicEnergySmooth2010all_neg.m
    - Function that describes the metabolic energy model from Umberger (2010) treating negative mechanical work as Umberger et al. (2003).
7. getPctSTSmooth.m
    - Function that describes the orderly recruitment model.
8. getSlowTwitchRatios.m
    - Helper function that returns the percentage of slow twitch fibers in the muscles.
9. getSpecificTensions.m
    - Helper function that returns the specific tension of the muscles.
    
### PassiveMoments

Folder with the script describing the passive joint torques.

1. PassiveMomentsData.m
    - Script that describes the passive joint torques.
    
### CasADiFunctions

Folder with scripts implementing several CasADi-based functions.

1. CasADiFunctions_all.m
    - Script for use in PredSim_all.m.
2. CasADiFunctions_prosthesis.m
    - Script for use in PredSim_prosthesis.m.
3. CasADiFunctions_tracking.m
    - Script for use in TrackSim.m.
 
### Contact

Folder with the script describing the foot-ground contact model (for use in tracking simulation).

1. HCContactModel.m
    - Function that describes the Hunt-Crossley foot-ground contact model.
  
### CollocationScheme

Folder with the script describing the collocation scheme.

1. CollocationScheme.m
    - Script that describes the collocation scheme.
  
### Bounds

Folder with the scripts setting the bounds of the optimal control problems.

1. getBounds_all.m
    - Script for use in PredSim_all.m.
2. getBounds_prosthesis.m
    - Script for use in PredSim_prosthesis.m.
3. getBounds_tracking.m
    - Script for use in TrackSim.m.
  
### IG (Initial Guesses)

Folder with the scripts setting the initial guesses of the optimal control problems.

1. getGuess_QR.m
    - Script that describes the quasi-random initial guess.
2. getGuess_QR_prosthesis.m
    - Script that describes the quasi-random initial guess for the prosthesis case.
3. getGuess_DI.m
    - Script that describes the data-informed initial guess.
4. getGuess_DI_prosthesis.m
    - Script that describes the data-informed initial guess for the prosthesis case.
5. getGuess_DI_t.m
    - Script that describes the data-informed initial guess with the final time based on provided data.
6. getGuess_DI_tracking.m
    - Script that describes the data-informed initial guess for the tracking simulation.
 
### Plots

Folder with scripts plotting initial guesses versus bounds.

1. plot_BoundsVSInitialGuess_all.m
    - Script for use in PredSim_all.m.
2. plot_BoundsVSInitialGuess_prosthesis.m
    - Script for use in PredSim_prosthesis.m.
3. plot_BoundsVSInitialGuess_tracking.m
    - Script for use in TrackSim.m.
  
### VariousFunctions

Folder with scripts with various purposes.

1. barwitherr.m
    - Function that generates bar plots with error bars.
2. getGRF.m
    - Function that extracts ground reaction forces.
3. getID.m
    - Function that extracts inverse dynamics results.
4. getIK.m
    - Function that extracts inverse kinematics results.
5. readDiary.m
    - Function that reads diaries from IPOPT.
6. write_motionFile.m
    - Function that writes motion files for OpenSim.
7. SplineEval_ppuval.m
    - Script that evaluates splines.
   
### ResultsAnalysis

Folder with scripts reproducing all figures of the study.

1. Fig1
    - Script that reproduces Fig 1.
2. Fig2
    - Script that reproduces Fig 2.
3. Fig3A
    - Script that reproduces Fig 3A.
4. Fig3B
    - Script that reproduces Fig 3B.
5. Fig4
    - Script that reproduces Fig 4.
6. FigS1
    - Script that reproduces Fig S1.
7. FigS2
    - Script that reproduces Fig S2.
8. FigS3
    - Script that reproduces Fig S3.
9. FigS4
    - Script that reproduces Fig S4.
10. FigS5
    - Script that reproduces Fig S5.
11. FigS6
    - Script that reproduces Fig S6.
12. getCPU_all.m
    - Script that extracts the CPU times from the simulation results.
13. observeResults_PredSim.m
    - Scripts that plot results not presented in the paper.
14. predSim_data_all.m
    - Script that loads the simulation results.
15. predSim_settings_all.m
    - Script that loads the settings of the optimal control problems.
   
### OpenSimModel\subject1

Folder with data from subject1.
    
1. GRF
    - GRF_gait_1.mat
        - Ground reaction forces (for use in tracking simulation). 
2. ID
    - ID_gait_1.mat
        - Joint torques (for use in tracking simulation).
3. IK
    1. IK_average_running_HGC.mat
        - Joint positions from running, averaged across experimental trials, for half a gait cycle.
        - For use with data-informed (running) initial guess.
    2. IK_average_walking_FGC.mat
        - Joint positions from walking, averaged across experimental trials, for a full a gait cycle.
        - For use with data-informed (walking) initial guess: prosthesis case.
    3. IK_average_walking_HGC.mat
        - Joint positions from walking, averaged across experimental trials, for half a gait cycle.
        - For use with data-informed (walking) initial guess.
    4. IK_gait_1.mat
        - Joint positions (for use in tracking simulation).
4. subject1.osim
    - OpenSim model used in this study.
 
### ExperimentalData

Folder with experimental data.

1. ExperimentalData.mat
    - Structure with experimental data: more information in the readme of that folder. 
        
### Results

Folder with simulation results.

1. PredSim_all/Results_all.mat
    - Structure with all results from the predictive simulations except those with the prosthesis.
2. PredSim_prosthesis/Results_prosthesis.mat
    - Structure with results from the predictive simulations with the prosthesis.
3. TrackSim/Results_tracking.mat
    - Structure with results from the tracking simulations.
        
### ExternalFunctions

Folder with external functions, more information in the readme of that folder. 
