% This script reproduces Fig 3 (A)
% Author: Antoine Falisse
% Date: 1/7/2019

clear all
close all
clc

%% Settings
% Selected trials
% 1:    nominal cost function
% 119:  model with hip actuators weakened by 50 % 
% 120:  model with hip actuators weakened by 75 % 
% 123:  model with hip actuators weakened by 90 % 
ww  = [1,119,120,123]; 
% Fixed settings
subject = 'subject1';
body_mass = 62;
body_weight = 62*9.81;
setup.derivatives = 'AD';
% Load pre-defined settings
pathmain = pwd;
[pathrepo,~,~] = fileparts(pathmain);
pathOCP = [pathrepo,'/OCP'];
addpath(genpath(pathOCP));
settings = getSettings_predSim_all();

%% Load results
% Pre-allocation structures
Qs_opt              = struct('m',[]);
Qdots_opt           = struct('m',[]);
Acts_opt            = struct('m',[]);
GRFs_opt            = struct('m',[]);
Ts_opt              = struct('m',[]);
COT_opt             = struct('m',[]);
StrideLength_opt    = struct('m',[]);
StepWidth_opt       = struct('m',[]);
TrunkSway_opt       = struct('m',[]);
Cost                = struct('m',[]);
CPU_tot             = struct('m',[]);
% Loop over cases
for k = 1:length(ww)
    predSim_data_all;
    % Extract also trunk sway measures, obtained post=processing
    TrunkSway_opt(ww(k)).m = Results_all.(['Speed_',num2str(v_tgt_id*100)]). ...  
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
        (['CoContraction_',num2str(coCont)]).TrunkSway_opt;
end

%% Load reference data
pathReferenceData = [pathrepo,'/ExperimentalData'];
load([pathReferenceData,'/ExperimentalData.mat'],'ExperimentalData');

%% Common settings for plots
label_fontsize  = 16;
line_linewidth  = 3;
% Colors
color_all(1,:) = [0,0,0];
color_all(2,:) = [72,133,237]/255;  % Blue
color_all(3,:) = [219,50,54]/255;   % Red
color_all(4,:) = [244,194,13]/255;  % Yellow
color_exp      = [168,168,168]/255; % Grey

%% Plot joint kinematics
Qref = ExperimentalData.Q;
pos_Qs = 3:4;
ylim_Qs = [-40,-20;40,20];
idx_Qs_weak = [10,11];
NumTicks_YQs = 3;
NumTicks_XQs = 2;
joints_ref_weak = {'hip_flexion','hip_adduction'};
joints_tit = {'Hip flexion','Hip adduction'};
figure()
for i = 1:length(idx_Qs_weak)
    subplot(3,4,pos_Qs(i))
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Qs_opt(ww(k)).m,1)-1):100;
        plot(x,Qs_opt(ww(k)).m(:,idx_Qs_weak(i)),...
            'color',color_all(k,:),'linewidth',line_linewidth);
        hold on;    
    end
    % Experimental data
    idx_jref = strcmp(Qref.(subject).Qs.colheaders,joints_ref_weak{i});
    meanPlusSTD = Qref.(subject).Qs.mean(:,idx_jref) + 2*Qref.(subject).Qs.std(:,idx_jref);
    meanMinusSTD = Qref.(subject).Qs.mean(:,idx_jref) - 2*Qref.(subject).Qs.std(:,idx_jref);          
    stepQ = (size(Qs_opt(ww(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
    intervalQ = 1:stepQ:size(Qs_opt(ww(k)).m,1);
    sampleQ = 1:size(Qs_opt(ww(k)).m,1);
    meanPlusSTD = interp1(intervalQ,meanPlusSTD,sampleQ);
    meanMinusSTD = interp1(intervalQ,meanMinusSTD,sampleQ);
    fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
    alpha(.25);   
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);
    title(joints_tit{i},'Fontsize',label_fontsize); 
    % Y-axis
    ylim([ylim_Qs(1,i) ylim_Qs(2,i)]);
    L = get(gca,'YLim');
    set(gca,'YTick',linspace(L(1),L(2),NumTicks_YQs));   
    % X-axis    
    set(gca,'XTick',[]);  
    box off;
end 

%% Plot joint kinetics
IDref = ExperimentalData.Torques;
ylim_Qdots = [-70,-100;70,30];
pos_Qdots = [7,8];
for i = 1:length(idx_Qs_weak)
    subplot(3,4,pos_Qdots(i))
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Ts_opt(ww(k)).m,1)-1):100;
        plot(x,Ts_opt(ww(k)).m(:,idx_Qs_weak(i))*body_mass,...
            'color',color_all(k,:),'linewidth',line_linewidth);
        hold on;    
    end
    % Experimental data
    idx_jref = strcmp(IDref.(subject).colheaders,joints_ref_weak{i});
    meanPlusSTD = IDref.(subject).mean(:,idx_jref) + 2*IDref.(subject).std(:,idx_jref);
    meanMinusSTD = IDref.(subject).mean(:,idx_jref) - 2*IDref.(subject).std(:,idx_jref);  
    stepID = (size(Ts_opt(ww(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
    intervalID = 1:stepID:size(Ts_opt(ww(k)).m,1);
    sampleID = 1:size(Ts_opt(ww(k)).m,1);
    meanPlusSTD = interp1(intervalID,meanPlusSTD,sampleID);
    meanMinusSTD = interp1(intervalID,meanMinusSTD,sampleID); 
    hold on
    fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
    alpha(.25);
    % Plot settings
    set(gca,'Fontsize',label_fontsize);
    % Y-axis
    ylim([ylim_Qdots(1,i) ylim_Qdots(2,i)]);
    L = get(gca,'YLim');
    set(gca,'YTick',[L(1),0,L(2)]);  
    if i == 1
        ylabel('Torque (Nm)','Fontsize',label_fontsize);
    end    
    % X-axis    
    L = get(gca,'XLim');
    set(gca,'XTick',linspace(L(1),L(2),NumTicks_XQs))
    xlabel('Gait cycle (%)','Fontsize',label_fontsize);      
    box off;
end

%% Plot step widths
pathVariousFunctions = [pathrepo,'/VariousFunctions'];
addpath(genpath(pathVariousFunctions)); 
StepWidthref = ExperimentalData.StepWidth;
% Post-processing for barplot
stride_width_all.std = [StepWidthref.(subject).std,zeros(1,length(ww))];
stride_width_all.mean = [StepWidthref.(subject).mean,zeros(1,length(ww))];
for k = 1:length(ww)
    stride_width_all.mean(1,k+1) = mean(StepWidth_opt(ww(k)).m);
end
stride_width_all.std = [stride_width_all.std;zeros(1,length(stride_width_all.std))];
stride_width_all.mean = [stride_width_all.mean;zeros(1,length(stride_width_all.mean))];
% Plot
ylim_width = [0,0.4];
xlim_width = [0.4 1.6];
pos_width = 6;
NumTicks_width = 2;
subplot(3,4,pos_width)
h = barwitherr(stride_width_all.std,stride_width_all.mean);
% Plot settings 
set(gca,'Fontsize',label_fontsize);
title('Step width','Fontsize',label_fontsize);
% X-axis
xlim([xlim_width(1,1) xlim_width(1,2)]);
set(gca,'XTick',[]);
% Y-axis
ylim([ylim_width(1,1) ylim_width(1,2)]);
L = get(gca,'YLim');
set(gca,'YTick',linspace(L(1),L(2),NumTicks_width));  
ylabel('Distance (m)','Fontsize',label_fontsize);
% Colors
set(h(1),'FaceColor',color_exp);
for k = 1:length(ww)
    set(h(k+1),'FaceColor',color_all(k,:));
end
box off;

%% Plot trunk sway
TrunkSwayref = ExperimentalData.TrunkSway;
pos_trunk = 2;
ylim_trunk = [-10,10];
x = 1:100;
NumTicks_trunkY = 3;
NumTicks_trunkX = 2;
subplot(3,4,pos_trunk)
% Simulation results
for k = 1:length(ww)
    plot(x,TrunkSway_opt(ww(k)).m,'color',color_all(k,:),...
        'linewidth',line_linewidth);          
    hold on;        
end
% Experimental data
meanPlusSTD = TrunkSwayref.(subject).mean + 2*TrunkSwayref.(subject).std;
meanMinusSTD = TrunkSwayref.(subject).mean - 2*TrunkSwayref.(subject).std;          
stepQ = (size(Qs_opt(ww(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
intervalQ = 1:stepQ:size(Qs_opt(ww(k)).m,1);
sampleQ = 1:size(Qs_opt(ww(k)).m,1);
meanPlusSTD = interp1(intervalQ,meanPlusSTD,sampleQ);
meanMinusSTD = interp1(intervalQ,meanMinusSTD,sampleQ);
fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
alpha(.25);    
% Plot settings     
set(gca,'Fontsize',label_fontsize);    
title('Trunk sway','Fontsize',label_fontsize);  
% Y-axis
ylim([ylim_trunk(1) ylim_trunk(2)]);
L = get(gca,'YLim');
set(gca,'YTick',linspace(L(1),L(2),NumTicks_trunkY));       
ylabel('Angle (°)','Fontsize',label_fontsize);
% X-axis
L = get(gca,'XLim');
set(gca,'XTick',linspace(L(1),L(2),NumTicks_trunkX))
xlabel('Gait cycle (%)','Fontsize',label_fontsize);
box off;
