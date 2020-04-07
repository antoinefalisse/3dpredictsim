OpenSimModel\subject1
=====================

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
    - OpenSim model used in the [paper](http://dx.doi.org/10.1098/rsif.2019.0402).
5. subject1_v2.osim
    - OpenSim model used from v2.0. Only slight changes with respect to subject1.osim to better match model built in external function.
    