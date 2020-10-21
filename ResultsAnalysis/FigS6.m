% This script reproduces Fig S6
% Author: Antoine Falisse
% Date: 1/7/2019

clear all
close all
clc

%% Settings
% Selected trials
% 1:    quasi-random initial guess with 50 mesh intervals
% 22:   data-informed (walking) initial guess with 50 mesh intervals
% 43:   data-informed (running) initial guess with 50 mesh intervals
% 116:  quasi-random initial guess with 100 mesh intervals
% 117:  data-informed (walking) initial guess with 100 mesh intervals
ww  = [1,22,43,116,117]; 
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
Cost                = struct('m',[]);
CPU_tot             = struct('m',[]);
% Loop over cases
for k = 1:length(ww)
    predSim_data_all;
end

%% Load reference data
pathReferenceData = [pathrepo,'/ExperimentalData'];
load([pathReferenceData,'/ExperimentalData.mat'],'ExperimentalData');

%% Common settings for plots
label_fontsize  = 16;
line_linewidth  = 3;
% Colors
color_all(1,:) = [0,0,0];
color_all(2,:) = [72,133,237]/255;     % Blue
color_all(3,:) = [137,10,255]/255;     % Purple
color_all(4,:) = [244,194,13]/255;     % Yellow
color_all(5,:) = [219,50,54]/255;      % Red
color_exp       = [168,168,168]/255;   % Grey
line_linestyle = {'-','-',':','-','-'};

%% Plot joint kinematics
Qref = ExperimentalData.Q;
pos_Qs = 1:4;
ylim_Qs = [-50,50;-20,20;-80,20;-40,40];
idx_Qs = [10,11,14,16];   
joints_ref = {'hip_flexion','hip_adduction',...
    'knee_angle','ankle_angle'};
joints_tit = {'Hip flexion','Hip adduction',...
    'Knee','Ankle'};
figure()
for i = 1:length(idx_Qs)
    subplot(3,4,pos_Qs(i))
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Qs_opt(ww(k)).m,1)-1):100;
        plot(x,Qs_opt(ww(k)).m(:,idx_Qs(i)),...
            'color',color_all(k,:),'linewidth',line_linewidth,'linestyle',line_linestyle{k});
        hold on;    
    end
    % Experimental data
    idx_jref = strcmp(Qref.(subject).Qs.colheaders,joints_ref{i});
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
    title(joints_tit{(i)},'Fontsize',label_fontsize);  
    % Y-axis
    ylim([ylim_Qs(i,1) ylim_Qs(i,2)]);
    L = get(gca,'YLim');
    set(gca,'YTick',[L(1),0,L(2)]);       
    if i == 1
        ylabel('Angle (°)','Fontsize',label_fontsize);
    end      
    % X-axis
    set(gca,'XTick',[]);
    box off;
end 

%% Plot joint kinetics
Tref = ExperimentalData.Torques;
ylim_T = [-100,100;-100,100;-100,100;-200,100];
pos_T = 5:8;
NumTicks_XT = 2;
for i = 1:length(idx_Qs)
    subplot(3,4,pos_T(i))
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Ts_opt(ww(k)).m,1)-1):100;
        plot(x,Ts_opt(ww(k)).m(:,idx_Qs(i))*body_mass,...
                'color',color_all(k,:),'linewidth',line_linewidth,'linestyle',line_linestyle{k});
        hold on;    
    end
    % Experimental data
    idx_jref = strcmp(Tref.(subject).colheaders,joints_ref{i});
    meanPlusSTD = Tref.(subject).mean(:,idx_jref) + 2*Tref.(subject).std(:,idx_jref);
    meanMinusSTD = Tref.(subject).mean(:,idx_jref) - 2*Tref.(subject).std(:,idx_jref);  
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
    ylim([ylim_T(i,1) ylim_T(i,2)]);
    L = get(gca,'YLim');
    set(gca,'YTick',[L(1),0,L(2)]);       
    if i == 1
        ylabel('Torque (Nm)','Fontsize',label_fontsize);
    end      
    % X-axis
    if i == 4 
        L = get(gca,'XLim');
        set(gca,'XTick',linspace(L(1),L(2),NumTicks_XT))
        xlabel('Gait cycle (%)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end
    box off;
end 

%% Plot ground reaction forces
GRFref = ExperimentalData.GRFs;
grf_names = {'x','y','z'};
GRF_str = {'Fore-aft','Vertical','Lateral'};
ylim_GRF = ([-50,50;0 220;-25 25]);
NumTicks_YGRF = 3;
NumTicks_XGRF = 2;
for i = 1:length(GRF_str)
    subplot(3,4,i+8)
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(GRFs_opt(ww(k)).m,1)-1):100;
        plot(x,GRFs_opt(ww(k)).m(:,i),'color',...
                color_all(k,:),'linewidth',line_linewidth,'linestyle',line_linestyle{k});
        hold on;           
    end    
    meanPlusSTD = GRFref.(subject).mean(:,i) + 2*GRFref.(subject).std(:,i);    
    meanMinusSTD = GRFref.(subject).mean(:,i) - 2*GRFref.(subject).std(:,i);   
    % Experimental data
    stepGRF = (size(GRFs_opt(ww(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
    intervalGRF = 1:stepGRF:size(GRFs_opt(ww(k)).m,1);
    sampleGRF = 1:size(GRFs_opt(ww(k)).m,1);
    meanPlusSTD = interp1(intervalGRF,meanPlusSTD,sampleGRF);
    meanMinusSTD = interp1(intervalGRF,meanMinusSTD,sampleGRF);
    hold on
    fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');     
    alpha(.25);    
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);    
    title(GRF_str{(i)},'Fontsize',label_fontsize);  
    % Y-axis
    ylim([ylim_GRF(i,1) ylim_GRF(i,2)]);
    L = get(gca,'YLim');
    set(gca,'YTick',linspace(L(1),L(2),NumTicks_YGRF));       
    if i == 1
        ylabel('Force (%BW)','Fontsize',label_fontsize);
    end      
    % X-axis
    L = get(gca,'XLim');
    set(gca,'XTick',linspace(L(1),L(2),NumTicks_XGRF))
    xlabel('Gait cycle (%)','Fontsize',label_fontsize);
    box off;
end      

%% Plot metabolic cost of transport (COT)
pathVariousFunctions = [pathrepo,'/VariousFunctions'];
addpath(genpath(pathVariousFunctions));
% Reference data from Miller et al. (2014)
COTref.mean = 3.35;
COTref.std = 2*0.25; 
% Post-processing for barplot
COT_order = zeros(1,length(ww));
for k = 1:length(ww)
    COT_order(k) = COT_opt(ww(k)).m;
end
COT_order_mean = COT_order;
COT_order_mean = [COTref.mean,COT_order_mean];
COT_order_mean = [COT_order_mean;zeros(1,length(COT_order_mean))];
COT_order_std = zeros(2,length(COT_order_mean));
COT_order_std(1,1) = COTref.std;
% Plot
ylim_COT = [0,6];
xlim_COT = [0.4 1.6];
NumTicks_COT = 2;
subplot(3,4,12)
h_COT = barwitherr(COT_order_std,COT_order_mean);
% Plot settings 
set(gca,'Fontsize',label_fontsize);
title('COT','Fontsize',label_fontsize);
% X-axis
xlim([xlim_COT(1,1) xlim_COT(1,2)]);
set(gca,'XTick',[]);
% Y-axis
ylim([ylim_COT(1,1) ylim_COT(1,2)]);
L = get(gca,'YLim');
set(gca,'YTick',linspace(L(1),L(2),NumTicks_COT));
ylabel('(J kg-1 m-1)','Fontsize',label_fontsize);
% Colors
set(h_COT(1),'FaceColor',color_exp);
color_COT = color_all;
for k = 1:length(ww)
    set(h_COT(k+1),'FaceColor',color_COT(k,:));
end
box off;