% This script reproduces Fig S2
%
% WARNING: after publication, we realized that we made an error in the
% following models: Umberger2003, Umberger2010, Uchida2016 (see CHANGELOG
% for details). This bug has been corrected now. If you re-run the simulations
% and run this script, you might therefore get a different figure than the one
% in the paper. The results saved in Results_all still contain the original
% results (except if you overwrote them).
%
% Author: Antoine Falisse
% Date: 1/7/2019

clear all
close all
clc

%% Settings
% Selected trials
% 1:    Bhargava et al. (2004)
% 187:  Uchida et al. (2016)
% 184:  Umberger et al. (2003)
% 186:  Umberger (2010)
ww  = [1,187,184,186]; 
% 184:  Umberger et al. (2003)
% 186:  Umberger (2010)
% 189:  Umberger (2010) treating lengthening heat as Umberger et al. (2003)
% 192:  Umberger (2010) treating mechanical work as Umberger et al. (2003)
wwb  = [184,186,189,192]; % specific analysis on Umberger 2010
ww = [ww,wwb];
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
color_all(1,:) = [0,0,0];           % Black
color_all(2,:) = [72,133,237]/255;  % Blue
color_all(3,:) = [244,194,13]/255;  % Yellow
color_all(4,:) = [219,50,54]/255;   % Red
color_exp       = [168,168,168]/255;    % Grey
lineS_s = {'-','-','--',':'};
color_s(1,:) = [244,194,13]/255;    % Yellow
color_s(2,:) = [219,50,54]/255;     % Red
color_s(3,:) = [137,10,255]/255;    % Purple
color_s(4,:) = [3,221,255]/255;     % Cyan

%% Plot joint kinematics
Qref = ExperimentalData.Q;
ylim_Qs = [-50,-80,-40;50,20,40];
idx_plot_Qs = 1:3;
Nmodels = 4;
idx_Qs_mE = [10,14,16];
joints_ref_mE = {'hip_flexion','knee_angle','ankle_angle'};
joints_tit_mE = {'Hip flexion','Knee','Ankle'};
figure()
for i = 1:length(idx_Qs_mE)
    subplot(3,4,idx_plot_Qs(i))
    % Simulation results
    for k = 1:Nmodels
        x = 1:(100-1)/(size(Qs_opt(ww(k)).m,1)-1):100;
        plot(x,Qs_opt(ww(k)).m(:,idx_Qs_mE(i)),...
            'color',color_all(k,:),'linewidth',line_linewidth);
        hold on;    
    end
    % Experimental data
    idx_jref = strcmp(Qref.(subject).Qs.colheaders,joints_ref_mE{i});
    meanPlusSTD = Qref.(subject).Qs.mean(:,idx_jref) + 2*Qref.(subject).Qs.std(:,idx_jref);
    meanMinusSTD = Qref.(subject).Qs.mean(:,idx_jref) - 2*Qref.(subject).Qs.std(:,idx_jref);          
    stepQ = (size(Qs_opt(ww(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
    intervalQ = 1:stepQ:size(Qs_opt(ww(k)).m,1);
    sampleQ = 1:size(Qs_opt(ww(k)).m,1);
    meanPlusSTD = interp1(intervalQ,meanPlusSTD,sampleQ);
    meanMinusSTD = interp1(intervalQ,meanMinusSTD,sampleQ);
    hold on
    fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
    alpha(.25);
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);   
    title(joints_tit_mE{i},'Fontsize',label_fontsize);    
    % X-axis    
    set(gca,'XTick',[]);           
    % Y-axis
    ylim([ylim_Qs(1,i) ylim_Qs(2,i)]);
    L = get(gca,'YLim');
    set(gca,'YTick',[L(1),0,L(2)]); 
    if i == 1    
        ylabel('Angle (°)','Fontsize',label_fontsize);
    end
    box off;
end       

%% Plot metabolic cost of transport (COT)
pathVariousFunctions = [pathrepo,'/VariousFunctions'];
addpath(genpath(pathVariousFunctions));
% Reference data from Miller et al. (2014)
COTref.mean = 3.35;
COTref.std = 2*0.25; 
% Post-processing for barplot
COT_order = zeros(1,Nmodels);
for k = 1:Nmodels
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
NumTicks_COT = 3;
subplot(3,4,4)
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
for k = 1:Nmodels
    set(h_COT(k+1),'FaceColor',color_COT(k,:));
end
box off;

%% Plot joint kinematics: sensitivity analysis
idx_plot_Qs_s = 5:7;
Nmodels_s = 4;
NumTicks_XQs = 2;
for i = 1:length(idx_Qs_mE)
    subplot(3,4,idx_plot_Qs_s(i))
    % Simulation results
    for k = 1:Nmodels_s
        x = 1:(100-1)/(size(Qs_opt(wwb(k)).m,1)-1):100;
        plot(x,Qs_opt(wwb(k)).m(:,idx_Qs_mE(i)),...
            'color',color_s(k,:),'linestyle',lineS_s{k},'linewidth',line_linewidth);
        hold on;    
    end
    % Experimental data
    idx_jref = strcmp(Qref.(subject).Qs.colheaders,joints_ref_mE{i});
    meanPlusSTD = Qref.(subject).Qs.mean(:,idx_jref) + 2*Qref.(subject).Qs.std(:,idx_jref);
    meanMinusSTD = Qref.(subject).Qs.mean(:,idx_jref) - 2*Qref.(subject).Qs.std(:,idx_jref);          
    stepQ = (size(Qs_opt(wwb(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
    intervalQ = 1:stepQ:size(Qs_opt(wwb(k)).m,1);
    sampleQ = 1:size(Qs_opt(wwb(k)).m,1);
    meanPlusSTD = interp1(intervalQ,meanPlusSTD,sampleQ);
    meanMinusSTD = interp1(intervalQ,meanMinusSTD,sampleQ);
    hold on
    fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
    alpha(.25);
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);  
    % X-axis    
    L = get(gca,'XLim');
    set(gca,'XTick',linspace(L(1),L(2),NumTicks_XQs))
    xlabel('Gait cycle (%)','Fontsize',label_fontsize);           
    % Y-axis
    ylim([ylim_Qs(1,i) ylim_Qs(2,i)]);
    L = get(gca,'YLim');
    set(gca,'YTick',[L(1),0,L(2)]); 
    if i == 1    
        ylabel('Angle (°)','Fontsize',label_fontsize);
    else
        
    end
    box off;
end   
