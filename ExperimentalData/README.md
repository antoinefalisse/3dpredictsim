ExperimentalData.mat
==================== 

ExperimentalData is a nested structure that contains data from the right leg of subject1 from 11 walking trials:
    
1. Q.subject1
    1. Qs         	    
        - joint positions
    2. Qdots      	    
        - joint velocities	
2. GRFs.subject1          
    - ground reaction forces
3. Torques.subject1       
    - joint torques 
4. Powers.subject1        
    - joint powers 
5. EMG.subject1           
    - electromyography: NaN when not available
6. TrunkSway.subject1     
    - trunk sway
7. StepWidth.subject1     
    - step widths
8. StrideLength.subject1  
    - stride lengths
    
Each variable includes raw and average data:   
1. all    	    
    - data from all trials
2. mean   	    
    - mean across trials
3. std    	    
    - standard deviations
4. colheaders 	
    - headers of the columns 
    
All time-dependent data are interpolated over 100 points describing a full gait cycle starting at heel strike of the right leg.	
    
ExperimentalData also contains data from walking trials with a prosthesis from Quesada et al. (2016):
 
1. Quesada_2016_SuppData
    1. ankleTorqueData
        1. all    	
            - ankle torques of affected (prosthesis) leg over gait cycle for six subjects (100x6)
