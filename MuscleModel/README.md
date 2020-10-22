MuscleModel
===========

Folder with scripts and data describing muscle activation dynamics, muscle contraction dynamics, and arm activation dynamics. This folder also contains helper functions for use when formulating the optimal control problems.

1. Scripts:
    1. ArmActivationDynamics.m
        - Function that describes the dynamics of the arms, which are driven by torque actuators.        
    2. computeExcitationRaasch.m
        - Function that computes muscle excitations from time derivative of muscle activations. The muscle activation model is based on Raasch et al. (1991). More info in [De Groote et al. (2009)](https://doi.org/10.1080/10255840902788587).       
    3. FiberLength_TendonForce_tendon.m
        - Function that computes fiber lengths from muscle-tendon forces. The muscle model is based on [De Groote et al. (2016)](https://link.springer.com/article/10.1007/s10439-016-1591-9).         
    4. FiberVelocity_TendonForce_tendon.m
        - Function that computes fiber velocities from muscle-tendon forces. The muscle model is based on [De Groote et al. (2016)](https://link.springer.com/article/10.1007/s10439-016-1591-9).         
    5. ForceEquilibrium_FtildeState_all.m
        - Function that derives the Hill-equilibrium. The muscle model is based on [De Groote et al. (2016)](https://link.springer.com/article/10.1007/s10439-016-1591-9).        
    6. ForceEquilibrium_FtildeState_all_tendon.m
        - Function that derives the Hill-equilibrium with tendon stiffness as parameter. The muscle model is based on [De Groote et al. (2016)](https://link.springer.com/article/10.1007/s10439-016-1591-9).          
    7. MomentArmIndices.m        
        - Helper function that returns indices for use with the moment arms.        
    8. MuscleIndices.m
        - Helper function that returns indices for use with the muscles. 
    9. saveSubjectMTParameters.m
        - Script to extract and save muscle-tendon parameters based on a given OpenSim model.
    
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
    