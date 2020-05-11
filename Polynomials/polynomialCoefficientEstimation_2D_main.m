% To improve the computational efficiency of our simulations, we compute
% muscle-tendon lengths and moment arms based on joint coordinates using
% polynomial approximations. We optimize the polynomial coefficients based
% on data from a muscle analysis run in OpenSim using a dummy motion in
% which the model is positioned in many different poses.
%
% This script estimates the polynomial coefficients based on given input data.
%
% Authors: Wouter Aerts and Antoine Falisse
%
% Date: v1 - 03/04/2018, v2 - 25/03/2020
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

%% User inputs
runPolynomialCoefficientEstimation = 0; % set to 1 to estimate the polynomial coefficients.
saveQdot = 0; % set to 1 to generate and save random joint velocities.
saveResults = 0; % set to 1 to save the estimated polynomial coefficients.
evaluateResults = 1; % set to 1 to observe some results of the approximation.
subjectName = 'subject1_2D'; % subject name.
% This is the name of the folder in which you saved the results of the
% muscle analysis performed with your model and a dummy motion. An example
% dummy motion is given in Polynomials/MuscleAnalysis/dummy_motion/dummy_motion.mot. 
% You may want to adjust the motion file if your model has more degrees of
% freedom (e.g., muscle-driven mtp joint). You can also find an example setup
% file to run a muscle analysis with OpenSim: Polynomials/MuscleAnalysis/SetupMA_subject1.xml
% We assume that this folder is located in Polynomials/MuscleAnalysis/ResultsMA.
folderName_resultsMuscleAnalysis = ['ResultsMA_',subjectName]; 
% Those are the coordinates (i.e., degrees of freedom) of the model.
% Only these coordinates will be considered when estimating polynomial
% coefficients to approximate muscle-tendon lengths and moment arms. In
% this example, the model is symmetric and the same coefficients can thus
% be used for both left and right side muscles. Hence, we only compute the
% coefficients for the right side muscles. Note that the lumbar degrees of 
% freedom are not side-dependent and are actuated by six muscles. We therefore
% include these three degrees of freedom as part of the coordinate set and
% include the corresponding muscles as part of the muscle set.
coordinates = {'hip_flexion_r', 'knee_angle_r', 'ankle_angle_r'};
% Those are the muscles of the model for which we will approximate muscle-tendon
% lengths and moment arms through polynomials. Note that we only include
% the right side muscles (read previous comment) except for muscles
% actuating the lumbar degrees of freedom.
muscleNames = {'hamstrings_r', 'bifemsh_r', 'glut_max_r', 'iliopsoas_r', ...
    'rect_fem_r', 'vasti_r', 'gastroc_r', 'soleus_r', 'tib_ant_r'};

%% Extract coordinate positions from dummy motion
pathmain = pwd;
name_dummymotion = 'dummy_motion_gait1018.mot';
path_dummymotion = [pathmain,'/MuscleAnalysis/dummy_motion/'];
path_resultsMA = [pathmain,'/MuscleAnalysis/ResultsMA/',folderName_resultsMuscleAnalysis,'/'];
dummy_motion = importdata([path_dummymotion,name_dummymotion]);
idx_coordinates = 99*ones(1, length(coordinates));
for i = 1:length(coordinates)
    idx_coordinates(1,i) = find(strcmp(dummy_motion.colheaders,coordinates{i}));
end
if ~isempty(find(idx_coordinates==99,1))
    disp(['WARNING: ',...
        'There might be an issue when extracting data from the dummy motion.'])
end
q = dummy_motion.data(:,idx_coordinates).*(pi/180);
% Generate coordinate speeds between -1000 and 1000 (°/s).
% Those speeds are used to approximate muscle-tendon lengthening
% velocities. Coordinate speeds are not used for the estimation of
% polynomial coefficients.
if saveQdot
    a = -1000; b = 1000; r = [];
    for i = 1:length(coordinates)
        r1 = (b-a).*rand(size(q,1),1) + a;
        r = [r,r1];
    end
    dummy_qdot = r.*(pi/180);
    save([path_dummymotion,'dummy_qdot_gait1018.mat'],'dummy_qdot');
end
load([path_dummymotion,'dummy_qdot_gait1018.mat']);
qdot = dummy_qdot(:,:);

%% Import and structure data
% Muscle-tendon lengths (lMT)
lMT = importdata([path_resultsMA,'subject01_MuscleAnalysis_Length.sto']);
% Moment arms (MA)
for i = 1:length(coordinates)
    MA.([coordinates{i}]) = importdata([path_resultsMA,['subject01_MuscleAnalysis_MomentArm_',coordinates{i},'.sto']]);
end
if runPolynomialCoefficientEstimation
    MuscleData.dof_names = dummy_motion.colheaders(idx_coordinates);     
    MuscleData.muscle_names = muscleNames;
    for m = 1:length(muscleNames)
        MuscleData.lMT(:,m)     = lMT.data(:,strcmp(lMT.colheaders,muscleNames{m}));
        for i = 1:length(coordinates)
            MuscleData.dM(:,m,i) = MA.([coordinates{i}]).data(:,strcmp(lMT.colheaders,muscleNames{m}));
        end
    end
    MuscleData.q = q;
    MuscleData.qdot = qdot;
end

%% Estimate polynomial coefficents
if runPolynomialCoefficientEstimation
    [muscle_spanning_joint_INFO,MuscleInfo] = polynomialCoefficientEstimation(MuscleData);
    if saveResults
        save(['MuscleData_',subjectName,'.mat'], 'MuscleData');
        save(['muscle_spanning_joint_INFO_',subjectName,'.mat'], 'muscle_spanning_joint_INFO');
        save(['MuscleInfo_',subjectName,'.mat'], 'MuscleInfo');
    end
end

%% Evaluate results
if evaluateResults
    %% Create CasADi functions
    import casadi.*
    load(['muscle_spanning_joint_INFO_',subjectName,'.mat'])
    load(['MuscleInfo_',subjectName,'.mat'])
    NMuscle = length(MuscleInfo.muscle);
    q_leg_trunk = length(coordinates);
    qin     = SX.sym('qin',1,q_leg_trunk);
    qdotin  = SX.sym('qdotin',1,q_leg_trunk);
    lMT     = SX(NMuscle,1);
    vMT     = SX(NMuscle,1);
    dM      = SX(NMuscle,q_leg_trunk);
    for i=1:NMuscle     
        index_dof_crossing  = find(muscle_spanning_joint_INFO(i,:)==1);
        order               = MuscleInfo.muscle(i).order;
        [mat,diff_mat_q]    = n_art_mat_3_cas_SX(qin(1,index_dof_crossing),order);
        lMT(i,1)            = mat*MuscleInfo.muscle(i).coeff;
        vMT(i,1)            = 0;
        dM(i,1:q_leg_trunk) = 0;
        nr_dof_crossing     = length(index_dof_crossing); 
        for dof_nr = 1:nr_dof_crossing
            dM(i,index_dof_crossing(dof_nr)) = (-(diff_mat_q(:,dof_nr)))'*MuscleInfo.muscle(i).coeff;
            vMT(i,1) = vMT(i,1) + (-dM(i,index_dof_crossing(dof_nr))*qdotin(1,index_dof_crossing(dof_nr)));
        end 
    end
    f_lMT_vMT_dM = Function('f_lMT_vMT_dM',{qin,qdotin},{lMT,vMT,dM});

    %% Visualize results
    % This part should be adapted if you use a different model because the
    % indices and muscles might no longer hold.
    load(['MuscleData_',subjectName,'.mat']);
    lMT_out_r = zeros(size(q,1),NMuscle);
    vMT_out_r = zeros(size(q,1),NMuscle);
    dM_out_r = zeros(size(q,1),NMuscle,q_leg_trunk);
    for i = 1:size(q,1)
        [out1_r,out2_r,out3_r] = f_lMT_vMT_dM(MuscleData.q(i,:),MuscleData.qdot(i,:));
        lMT_out_r(i,:) = full(out1_r);
        vMT_out_r(i,:) = full(out2_r);
        for j = 1:q_leg_trunk
            dM_out_r(i,:,j) = full(out3_r(:,j));
        end
    end
    % Examples for a few muscle-tendon lengths
    figure()
    subplot(3,3,1)
    scatter3(MuscleData.q(:,1),MuscleData.q(:,2),lMT_out_r(:,1)); hold on;
    scatter3(MuscleData.q(:,1),MuscleData.q(:,2),MuscleData.lMT(:,1));
    xlabel('hip-flexion-r');
    ylabel('knee-angle-r');
    title('hamstrings-r');    
    subplot(3,3,2)
    scatter(MuscleData.q(:,2),lMT_out_r(:,2)); hold on;
    scatter(MuscleData.q(:,2),MuscleData.lMT(:,2));
    xlabel('knee-angle-r');
    title('bifemsh-r');
    subplot(3,3,3)
    scatter(MuscleData.q(:,1),lMT_out_r(:,3)); hold on;
    scatter(MuscleData.q(:,1),MuscleData.lMT(:,3));
    xlabel('hip-flexion-r');
    title('glut-max-r');
    subplot(3,3,4)
    scatter(MuscleData.q(:,1),lMT_out_r(:,4)); hold on;
    scatter(MuscleData.q(:,1),MuscleData.lMT(:,4));
    xlabel('hip-flexion-r');
    title('iliopsoas-r');
    subplot(3,3,5)    
    scatter3(MuscleData.q(:,1),MuscleData.q(:,2),lMT_out_r(:,5)); hold on;
    scatter3(MuscleData.q(:,1),MuscleData.q(:,2),MuscleData.lMT(:,5));
    xlabel('hip-flexion-r');
    ylabel('knee-angle-r');
    title('rect_fem_r');
    subplot(3,3,6)
    scatter(MuscleData.q(:,2),lMT_out_r(:,6)); hold on;
    scatter(MuscleData.q(:,2),MuscleData.lMT(:,6));
    xlabel('knee-angle-r');
    title('vasti_r');    
    subplot(3,3,7)
    scatter3(MuscleData.q(:,2),MuscleData.q(:,3),lMT_out_r(:,7)); hold on;
    scatter3(MuscleData.q(:,2),MuscleData.q(:,3),MuscleData.lMT(:,7));
    xlabel('knee-angle-r');
    ylabel('ankle-angle-r');
    title('gastroc_r');    
    subplot(3,3,8)
    scatter(MuscleData.q(:,3),lMT_out_r(:,8)); hold on;
    scatter(MuscleData.q(:,3),MuscleData.lMT(:,8));
    xlabel('ankle-angle-r');
    title('soleus-r');    
    subplot(3,3,9)
    scatter(MuscleData.q(:,3),lMT_out_r(:,9)); hold on;
    scatter(MuscleData.q(:,3),MuscleData.lMT(:,9));
    xlabel('ankle-angle-r');
    title('tib-ant-r');
    
    legend('Polynomial approximation','Model-based');
    suptitle('Muscle-tendon lengths');

    %% Assert results
    for i = 1:NMuscle  
        assertLMT(:,i) = abs(lMT_out_r(:,i) - MuscleData.lMT(:,i));
        for j = 1:length(coordinates)
            assertdM.([coordinates{j}])(:,i) = abs(dM_out_r(:,i,j) - MuscleData.dM(:,i,j));
        end
    end
    assertLMTmax_r = max(max(assertLMT));
    for j = 1:length(coordinates)
        assertdM.([coordinates{j},'_max']) = max(max(assertdM.([coordinates{j}])));
    end
end
