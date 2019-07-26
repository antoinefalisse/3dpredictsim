% This script reproduces Fig 4
% Author: Antoine Falisse
% Date: 1/7/2019

clear all
close all
clc

%% Settings
% Selected trial
% 1: nominal cost function
ww  = 1; 
% Fixed settings
subject = 'subject1';
body_mass = 8.84259166189724+2*6.98382288222561+2.78372323906632+...
    1.809420105393108+0.0750835667988218+0.048804318419234+...
    0.938544584985273+0.610053980240428+0.162631005686248+...
    0.105710153696061+25.7060604306454+2*1.52611854532613+...
    2*0.456132668302843+2*0.456132668302843+2*0.34350731810461;
body_weight = body_mass*9.81;
setup.derivatives = 'AD';
% Load pre-defined settings
settings = [        
    1.33, 4, 100, 500, 50000, 1000000, 1000, 2000, 2, 2, 1, 1, 0, 0, 0, 0, 0, 800; % 1
    % Quasi-random initial guess
    1.33, 4, 100, 500, 50000, 1000000, 1000, 2000, 2, 1, 1, 0, 0, 0, 0, 0, 0, 800; % 2
    ];

%% Load results
% Pre-allocation structures
Ts_opt_r = struct('m',[]);
Ts_opt_l = struct('m',[]);
COT_opt  = struct('m',[]);
% Loop over cases
for k = 1:length(ww)
    v_tgt       = settings(ww(k),1);    % target velocity
    tol_ipopt   = settings(ww(k),2);    % tolerance (means 1e-(tol_ipopt))
    N           = settings(ww(k),3);    % number of mesh intervals
    W.E         = settings(ww(k),4);    % weight metabolic energy
    W.Ak        = settings(ww(k),5);    % weight joint accelerations
    W.ArmE      = settings(ww(k),6);    % weight arm excitations
    W.passMom   = settings(ww(k),7);    % weight passive moments
    W.A         = settings(ww(k),8);    % weight muscle activations
    exp_E       = settings(ww(k),9);    % power metabolic energy
    IGsel       = settings(ww(k),10);   % initial guess selection
    cm          = settings(ww(k),11);   % contact model
    IGm         = settings(ww(k),12);   % initial guess mode
    IGcase      = settings(ww(k),13);   % initial guess case
    h_weak      = settings(ww(k),14);   % h_weakness hip actuators
    vMax_s      = settings(ww(k),15);   % maximal contraction velocity
    pf_weak     = settings(ww(k),16);   % weakness ankle plantaflexors
    mE          = settings(ww(k),17);   % metabolic energy model identifier
    kstiff      = settings(ww(k),18);   % prosthesis stiffness 
    v_tgt_id = round(v_tgt,2);
    % Load results
    pathmain = pwd;
    [pathRepo,~,~] = fileparts(pathmain);
    pathresults = [pathRepo,'\Results'];
    load([pathresults,'\Results_prosthesis.mat']);    
    % Unstructure data   
    Ts_opt_r(ww(k)).m = Results_prosthesis. ...
            (['Speed_',num2str(v_tgt_id*100)]). ...  
            (['W_MetabolicEnergyRate_',num2str(W.E)]). ...
            (['W_MuscleActivity_',num2str(W.A)]). ...            
            (['W_JointAcceleration_',num2str(W.Ak)]). ...
            (['W_PassiveTorque_',num2str(W.passMom)]). ...
            (['W_ArmExcitation_',num2str(W.ArmE)]). ...               
            (['Power_MetabolicEnergyRate_',num2str(exp_E)]). ...
            (['InitialGuessType_',num2str(IGsel)]). ...
            (['InitialGuessMode_',num2str(IGm)]). ...
            (['InitialGuessCase_',num2str(IGcase)]). ... 
            (['WeaknessHipActuators_',num2str(h_weak)]). ...
            (['WeaknessAnklePlantarflexors_',num2str(pf_weak)]). ...   
            (['MetabolicEnergyModel_',num2str(mE)]). ...            
            (['ContactModel_',num2str(cm)]). ...  
            (['Number_MeshIntervals_',num2str(N)]). ...
            (['MaximumContractionVelocity_',num2str(vMax_s)]). ...
            (['Stiffness_',num2str(kstiff)]).Ts_opt_r;
    Ts_opt_l(ww(k)).m = Results_prosthesis. ...
            (['Speed_',num2str(v_tgt_id*100)]). ...  
            (['W_MetabolicEnergyRate_',num2str(W.E)]). ...
            (['W_MuscleActivity_',num2str(W.A)]). ...            
            (['W_JointAcceleration_',num2str(W.Ak)]). ...
            (['W_PassiveTorque_',num2str(W.passMom)]). ...
            (['W_ArmExcitation_',num2str(W.ArmE)]). ...               
            (['Power_MetabolicEnergyRate_',num2str(exp_E)]). ...
            (['InitialGuessType_',num2str(IGsel)]). ...
            (['InitialGuessMode_',num2str(IGm)]). ...
            (['InitialGuessCase_',num2str(IGcase)]). ... 
            (['WeaknessHipActuators_',num2str(h_weak)]). ...
            (['WeaknessAnklePlantarflexors_',num2str(pf_weak)]). ...   
            (['MetabolicEnergyModel_',num2str(mE)]). ...            
            (['ContactModel_',num2str(cm)]). ...  
            (['Number_MeshIntervals_',num2str(N)]). ...
            (['MaximumContractionVelocity_',num2str(vMax_s)]). ...
            (['Stiffness_',num2str(kstiff)]).Ts_opt_l;
    COT_opt(ww(k)).m = Results_prosthesis. ...
            (['Speed_',num2str(v_tgt_id*100)]). ...  
            (['W_MetabolicEnergyRate_',num2str(W.E)]). ...
            (['W_MuscleActivity_',num2str(W.A)]). ...            
            (['W_JointAcceleration_',num2str(W.Ak)]). ...
            (['W_PassiveTorque_',num2str(W.passMom)]). ...
            (['W_ArmExcitation_',num2str(W.ArmE)]). ...               
            (['Power_MetabolicEnergyRate_',num2str(exp_E)]). ...
            (['InitialGuessType_',num2str(IGsel)]). ...
            (['InitialGuessMode_',num2str(IGm)]). ...
            (['InitialGuessCase_',num2str(IGcase)]). ... 
            (['WeaknessHipActuators_',num2str(h_weak)]). ...
            (['WeaknessAnklePlantarflexors_',num2str(pf_weak)]). ...   
            (['MetabolicEnergyModel_',num2str(mE)]). ...            
            (['ContactModel_',num2str(cm)]). ...  
            (['Number_MeshIntervals_',num2str(N)]). ...
            (['MaximumContractionVelocity_',num2str(vMax_s)]). ...
            (['Stiffness_',num2str(kstiff)]).COT_opt;
end

%% Load reference data
pathmain = pwd;
[pathrepo,~,~] = fileparts(pathmain);
pathReferenceData = [pathrepo,'\ExperimentalData'];
load([pathReferenceData,'\ExperimentalData.mat'],'ExperimentalData');

%% Common settings for plots
label_fontsize  = 16;
line_linewidth  = 3;
% Colors
color_all(1,:) = [219,50,54]/255; % Red

%% Plot joint kinetics
Tref = ExperimentalData.Torques;
% Process experimental data from Quesada et al . (2016)
ankleTorqueData.all = ExperimentalData.Quesada_2016_SuppData.ankleTorqueData.all*body_mass;
ankleTorqueData.mean = mean(ankleTorqueData.all,2);
ankleTorqueData.std = std(ankleTorqueData.all,[],2);
% Plot
idx_Qs = 16;
idx_Qs_l = 15;
joints_ref = {'pelvis_tilt','pelvis_list','pelvis_rotation',...
    'hip_flexion','hip_adduction','hip_rotation',...
    'knee_angle','ankle_angle','subtalar_angle',...
    'lumbar_extension','lumbar_bending','lumbar_rotation',...
    'arm_flex','arm_add','arm_rot','elbow_flex'};    
joints_tit = {'Ankle: prosthesis','Ankle: biological'};
NumTicks_XQs = 2;
ylim_Qs = [-150,50];
figure()
% Affected side (prosthesis)
subplot(3,4,2)
% Simulation results
k = 1;
x = 1:(100-1)/(size(Ts_opt_r(ww(k)).m,1)-1):100;
plot(x,Ts_opt_r(ww(k)).m(:,idx_Qs)*body_mass,'color',color_all(k,:),'linewidth',line_linewidth);
% Experimental data
meanPlusSTD = ankleTorqueData.mean + 2*ankleTorqueData.std;
meanMinusSTD = ankleTorqueData.mean - 2*ankleTorqueData.std;  
stepID = (size(Ts_opt_r(ww(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
intervalID = 1:stepID:size(Ts_opt_r(ww(k)).m,1);
sampleID = 1:size(Ts_opt_r(ww(k)).m,1);
meanPlusSTD = interp1(intervalID,meanPlusSTD,sampleID);
meanMinusSTD = interp1(intervalID,meanMinusSTD,sampleID); 
hold on
fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
alpha(.25);   
% Plot settings 
set(gca,'Fontsize',label_fontsize);
title(joints_tit{1},'Fontsize',label_fontsize);    
% X-axis    
L = get(gca,'XLim');
set(gca,'XTick',linspace(L(1),L(2),NumTicks_XQs))
xlabel('Gait cycle (%)','Fontsize',label_fontsize); 
% Y-axis 
ylim([ylim_Qs(1) ylim_Qs(2)]);
L = get(gca,'YLim');
set(gca,'YTick',[L(1),0,L(2)]); 
ylabel('Torque (Nm)','Fontsize',label_fontsize);
box off;
% Non-affected side (biological)  
subplot(3,4,3)  
% Simulation results
plot(x,Ts_opt_l(ww(k)).m(:,idx_Qs_l)*body_mass,'color',color_all(k,:),'linewidth',line_linewidth);
idx_jref = strcmp(Tref.(subject).colheaders,joints_ref{8});
% Experimental data
meanPlusSTD = Tref.(subject).mean(:,idx_jref) + 2*Tref.(subject).std(:,idx_jref);
meanMinusSTD = Tref.(subject).mean(:,idx_jref) - 2*Tref.(subject).std(:,idx_jref);  
stepID = (size(Ts_opt_r(ww(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
intervalID = 1:stepID:size(Ts_opt_r(ww(k)).m,1);
sampleID = 1:size(Ts_opt_r(ww(k)).m,1);
meanPlusSTD = interp1(intervalID,meanPlusSTD,sampleID);
meanMinusSTD = interp1(intervalID,meanMinusSTD,sampleID); 
hold on
fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
alpha(.25);
% Plot settings 
set(gca,'Fontsize',label_fontsize);
title(joints_tit{2},'Fontsize',label_fontsize);    
% X-axis    
L = get(gca,'XLim');
set(gca,'XTick',linspace(L(1),L(2),NumTicks_XQs))
xlabel('Gait cycle (%)','Fontsize',label_fontsize); 
% Y-axis 
ylim([ylim_Qs(1) ylim_Qs(2)]);
L = get(gca,'YLim');
set(gca,'YTick',[L(1),0,L(2)]); 
box off;
    
%% Plot metabolic cost of transport (COT)
pathVariousFunctions = [pathrepo,'\VariousFunctions'];
addpath(genpath(pathVariousFunctions)); 
% HARD CODED: value of COT from nominal simulation
COT_nom = 3.5464;
% Post-processing for barplot
 COT_order = zeros(1,length(ww));
for k = 1:length(ww)
    COT_order(k) = COT_opt(ww(k)).m;
end
% Set nominal as first
COT_order = [COT_nom,COT_order];
COT_order_mean = COT_order;
COT_order_mean = [COT_order_mean;zeros(1,length(COT_order_mean))];
COT_order_std = zeros(2,length(COT_order_mean));
% Plot
ylim_COT = [0,5];
xlim_COT = [0.4 1.6];
pos_COT = 4;
NumTicks_COT = 2;
subplot(3,4,pos_COT)
h_COT = barwitherr(COT_order_std,COT_order_mean);
% Plot settings 
set(gca,'Fontsize',label_fontsize);
% X-axis
xlim([xlim_COT(1,1) xlim_COT(1,2)]);
set(gca,'XTick',[]);
% Y-axis
ylim([ylim_COT(1,1) ylim_COT(1,2)]);
L = get(gca,'YLim');
set(gca,'YTick',linspace(L(1),L(2),NumTicks_COT));
ylabel('COT (J kg-1 m-1)','Fontsize',label_fontsize);
% Colors
color_COT(1,:) = [0,0,0];
color_COT(2,:) = color_all(1,:);
for k = 1:2
    set(h_COT(k),'FaceColor',color_COT(k,:));
end
box off;
