ExperimentalData.mat 

ExperimentalData is a nested structure that contains data from the right leg of subject1 from 11 walking trials:    
    .Q.subject1
        .Qs         	    joint positions
        .Qdots      	    joint velocities	
    .GRFs.subject1          ground reaction forces
    .Torques.subject1       joint torques 
    .Powers.subject1        joint powers 
    .EMG.subject1           electromyography: NaN when not available
    .TrunkSway.subject1     trunk sway
    .StepWidth.subject1     step widths
    .StrideLength.subject1  stride lengths
Each variable includes raw and average data:   
    .all    	    data from all trials
	.mean   	    mean across trials
	.std    	    standard deviations
	.colheaders 	headers of the columns 
All time-dependent data are interpolated over 100 points describing a full gait cycle starting at heel strike of the right leg.	
    
ExperimentalData also contains data from walking trials with a prosthesis from Quesada et al. (2016): 
    .Quesada_2016_SuppData
        .ankleTorqueData
            .all    	ankle torques of affected (prosthesis) leg over gait cycle for six subjects (100x6)
