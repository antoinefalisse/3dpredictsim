Results
=======

The results are divided into:

1. PredSim_all_v2/Results_all_v2.mat
    - Contains results from the predictive simulations produced with PredSim_all_v2.
2. PredSim_all/Results_all.mat
    - Contains all results from the predictive simulations except those with the prosthesis.
3. PredSim_prosthesis/Results_prosthesis.mat
    - Contains the results from the predictive simulations with the prosthesis.
4. TrackSimResults_tracking.mat 
    - Contains the results from the tracking simulations.

### Results_all is a nested structure with the following layers:

- Speed_< speed > 
    - where speed is the prescribed gait speed
- W_MetabolicEnergyRate_< WE > 
    - where WE is the weight on the metabolic energy rate term in the cost function
- W_MuscleActivity_< WA >                  
    - where WA is the weight on the muscle activity term in the cost function
- W_JointAcceleration_< WJAcc >            
    - where WJAcc is the weight on the joint acceleration term in the cost function
- W_PassiveTorque_< WPassT >               
    - where WPassT is the weight on the passive joint torque term in the cost function
- W_ArmExcitation_< WArmE >                
    - where WArmE is the weight on the arm excitation term in the cost function
- Power_MetabolicEnergyRate_< pE >         
    - where pE is the power of the metabolic energy rate term in the cost function 
- InitialGuessType_< IGT >                 
    - where IGT is the initial guess type: 
        - 1 quasi-random
        - 2 data-informed
- InitialGuessMode_< IGM >                 
    - where IGM is the data-informed initial guess mode:
        - 1 walking
        - 2 running 
        - 3 partial optimal solution at closest speed
        - 4 full optimal solution at closest speed
- InitialGuessCase_< IGC >                 
    - where IGC is the index of the trial whose solution is used as initial guess (IGM=3 || IGM=4)
- WeaknessHipActuators_< h_weak >          
    - where h_weak is the percentage of decrease in maximal isometric force of the hip muscles
- WeaknessAnklePlantarflexors_< pf_weak >  
    - where pf_weak is the percentage of decrease in maximal isometric force of the ankle plantarflexors
- MetabolicEnergyModel_< mE >              
    - where mE is the metabolic energy model identifier: 
        - 0 Bhargava et al. (2004)
        - 1 Umberger et al. (2003)
        - 2 Umberger (2010)
        - 3 Uchida et al. (2016)
        - 4 Umberger (2010) treating muscle lengthening heat rate as Umberger et al. (2003)
        - 5 Umberger (2010) treating negative mechanical work as Umberger et al. (2003)
- ContactModel_< CM >
    - where CM is the contact model identifier:
        - 1 generic
        - 2 subject-specific
- Number_MeshIntervals_< N >  
    - where N is the number of mesh intervals
- MaximumContractionVelocity_< vMax >      
    - where vMax is the maximal contraction velocity identifier:       
        - 0 generic (10*lMopt)
        - 1 doubled (20*lMopt)
- CoContraction_< CC >                     
    - where CC is co-contraction identifier:
        - 0 generic: bounds on muscle activations is 0.05
        - 1 low: bounds on muscle activations is 0.1
        - 2 medium: bounds on muscle activations is 0.15
        - 3 high: bounds on muscle activations is 0.2
- colheaders     
    - joints                             
        - headers of columns with joint-related results (1x29)
    - muscles                            
        - headers of columns with muscle-related results (1x92)
    - GRF                                
        - headers of columns with GRF-related results (1x6)  
                                            
### Results_all contains the following results:

- Qs_opt                 
    - joint positions (2*Nx29) (° or m)
- Qdots_opt              
    - joint velocities (2*Nx29) (°/s or m/s)
- GRFs_opt               
    - ground reaction forces (2*Nx6) (% body weight)
- Ts_opt                 
    - joint torques (2*Nx29) (Nm)
- Acts_opt               
    - muscle activations (2*Nx92)
- COT_opt                
    - metabolic cost of transport (J Kg-1 m-1)
- StrideLength_opt       
    - stride length (m)
- StepWidth_opt          
    - step width (m)
- CPU_IPOPT              
    - computational time spent in IPOPT (s)
- CPU_NLP                
    - computational time spent in evaluating the NLP functions (s)
- Cost                   
    - optimal cost
- OptSol                 
    - "1" if converged to an optimal solution
- TrunkSway_opt          
    - trunk sway (2*Nx1) (°). This is calculated post-processing through a body kinematics in OpenSim and corresponds to the rotation of the trunk in the frontal plane (only for trials for which it is used in figures).
- COM_p_y                
    - vertical position of the body COM. This is calculated post-processing through a body kinematics in OpenSim (only for trials for which it is used in figures).
- COM_v_x                
    - forward velocity of the body COM. This is calculated post-processing through a body kinematics in OpenSim (only for trials for which it is used in figures).

All results are for one full gait cycle starting at heel strike of the right leg.

### Results_prosthesis is a nested structure with the following layers:

- Speed_< speed >                          
    - where speed is the prescribed gait speed
- W_MetabolicEnergyRate_< WE >             
    - where WE is the weight on the metabolic energy rate term in the cost function
- W_MuscleActivity_< WA >                  
    - where WA is the weight on the muscle activity term in the cost function
- W_JointAcceleration_< WJAcc >            
    - where WJAcc is the weight on the joint acceleration term in the cost function
- W_PassiveTorque_< WPassT >               
    - where WPassT is the weight on the passive joint torque term in the cost function
- W_ArmExcitation_< WArmE >                
    - where WArmE is the weight on the arm excitation term in the cost function
- Power_MetabolicEnergyRate_<pE>         
    - where pE is the power of the metabolic energy rate term in the cost function 
- InitialGuessType_< IGT >                 
    - where IGT is the initial guess type: 
        - 1 quasi-random
        - 2 data-informed
- InitialGuessMode_< IGM >                 
    - where IGM is the data-informed initial guess mode: 
        - 1 walking
        - 2 running 
        - 3 partial optimal solution at closest speed
        - 4 full optimal solution at closest speed
- InitialGuessCase_< IGC >                 
    - where IGC is the index of the trial whose solution is used as initial guess (IGM=3 || IGM=4)
- WeaknessHipActuators_< h_weak >          
    - where h_weak is the percentage of decrease in maximal isometric force of the hip muscles
- WeaknessAnklePlantarflexors_< pf_weak >  
    - where pf_weak is the percentage of decrease in maximal isometric force of the ankle plantarflexors
- MetabolicEnergyModel_< mE >              
    - where mE is the metabolic energy model identifier: 
        - 0 Bhargava et al. (2004)
        - 1 Umberger et al. (2003)
        - 2 Umberger (2010)
        - 3 Uchida et al. (2016) 
        - 4 Umberger (2010) treating muscle lengthening heat rate as Umberger et al. (2003)
        - 5 Umberger (2010) treating negative mechanical work as Umberger et al. (2003)
- ContactModel_< CM >
    - where CM is the contact model identifier:
        - 1 generic
        - 2 subject-specific
- Number_MeshIntervals_< N >
    - where N is the number of mesh intervals
- MaximumContractionVelocity_< vMax >      
    - where vMax is the maximal contraction velocity identifier
        - 0 generic (10*lMopt)
        - 1 doubled (20*lMopt)
- Stiffness_< k >                          
    - where k is the torsional stiffness
- colheaders     
    - joints                             
        - headers of columns with joint-related results (1x29)
    - muscles                            
        - headers of columns with muscle-related results (1x80)
    - GRF                                
        - headers of columns with GRF-related results (1x6)  
    
### Results_prosthesis contains the following results:

- Qs_opt_r       
    - joint positions (Nx29) (° or m) for one full gait cycle starting at heel strike of the right leg                    
- Qs_opt_l       
    - joint positions (Nx29) (° or m) for one full gait cycle starting at heel strike of the left leg
- Qdots_opt_r
    - joint velocities (Nx29) (°/s or m/s) for one full gait cycle starting at heel strike of the right leg
- Qdots_opt_l
    - joint velocities (Nx29) (°/s or m/s) for one full gait cycle starting at heel strike of the left leg
- GRFs_opt_r
    - ground reaction forces (Nx6) (% body weight) for one full gait cycle starting at heel strike of the right leg
- GRFs_opt_l
    - ground reaction forces (Nx6) (% body weight) for one full gait cycle starting at heel strike of the left leg
- Ts_op_r
    - joint torques (Nx29) (Nm) for one full gait cycle starting at heel strike of the right leg
- Ts_opt_l
    - joint torques (Nx29) (Nm) for one full gait cycle starting at heel strike of the left leg
- Acts_opt_r
    - muscle activations (Nx80) for one full gait cycle starting at heel strike of the right leg
- Acts_opt_l
    - muscle activations (Nx80) for one full gait cycle starting at heel strike of the left leg
- COT_opt
    - metabolic cost of transport (J Kg-1 m-1)
- CPU_IPOPT
    - computational time spent in IPOPT (s)
- CPU_NLP
    - computational time spent in evaluating the NLP functions (s)
- Cost
    - optimal cost
- OptSol
    - "1" if converged to an optimal solution
    
### Results_tracking contains the following results:

- colheaders     
    - joints     
        - headers of columns with joint-related results (1x29)
    - GRF
        - headers of columns with GRF-related results (1x6)   
    - muscles
        - headers of columns with muscle-related results (1x92) 
    - paramsCM
        - headers of columns with parameter-related results (1x18)
            - loc: location
            - si: sphere i
            - r: right
            - x: forward direction
            - z: lateral direction
- Qs_opt         
    - joint positions (50x29) (° or m), the columns are ordered as colheaders.joints
- Ts_opt         
    - joint torques (50x29) (Nm), the columns are ordered as colheaders.joints
- GRFs_opt       
    - ground reaction forces (50x6) (N), the columns are ordered as colheaders.GRF
- GRMs_opt       
    - ground reaction moments (50x6) (Nm), the columns are ordered as colheaders.GRF
- Acts_opt       
    - muscle activations (50x92), the columns are ordered as colheaders.muscles 
- ParamsCM_opt   
    - optimized parameters of the contact models.
- Qs_toTrack     
    - experimental joint positions (to track) (50x30) (rad or m): the first column is time, the next columns are ordered as colheaders.joints
- Ts_toTrack      
    - experimental joint torques (to track) (50x30) (Nm): the first column is time, the next columns are ordered as colheaders.joints
- GRFs_toTrack   
    - experimental ground reaction forces (to track) (N) (50x7): the first column is time, the next columns are ordered as colheaders.joints
- GRMs_toTrack   
    - experimental ground reaction moments (to track) (Nm) (50x7): the first column is time, the next columns are ordered as colheaders.joints    
- CPU_IPOPT      
    - computational time spent in IPOPT (s)
- CPU_NLP        
    - computational time spent in evaluating the NLP functions (s)
- OptSol         
    - "1" if converged to an optimal solution
- ParamsCM_gen   
    - generic parameters of the contact models.
    