% This script allows ploting more results (all joint angles, velocities,
% torques, powers, ground reaction forces, all muscle activations, COT, 
% stride lengths, step, widths, CPU times, and optimal costs)
% Author: Antoine Falisse
% Date: 3/31/2020

clear all
close all
clc

%% Settings
% Select trials, for example
% 1:    nominal cost function
% 109:  metabolic energy rate at the first instead of second power wrt nominal cost function
% 104:  no metabolic energy rate term wrt nominal cost function
% 106:  no muscle activity term wrt nominal cost function
ww  = [22]; 
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
% user choice
showMainPlots = 1; % 0 to also see joint velocities and left m activations

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
CPU_IPOPT           = struct('m',[]);
CPU_NLP             = struct('m',[]);
legend_case         = cell(1,length(ww));
% Loop over cases
for k = 1:length(ww)
    predSim_data_all_v2;
    legend_case{k} = ['Case: ',num2str(ww(k))];
end

%% Helper names
joints_ref = {'pelvis_tilt','pelvis_list','pelvis_rotation',...
    'hip_flexion','hip_adduction','hip_rotation',...
    'knee_angle','ankle_angle','subtalar_angle',...
    'lumbar_extension','lumbar_bending','lumbar_rotation',...
    'arm_flex','arm_add','arm_rot','elbow_flex'};
joints_tit = {'Pelvis tilt','Pelvis list','Pelvis rotation','Pelvis tx',...
    'Pelvis ty','Pelvis tz','Hip flexion L','Hip adduction L',...
    'Hip rotation L','Hip flexion R','Hip adduction R','Hip rotation R',...
    'Knee L','Knee R','Ankle L','Ankle R',...
    'Subtalar L','Subtalar R',...
    'Lumbar extension','Lumbar bending','Lumbar rotation',...
    'Arm flexion L','Arm adduction L','Arm rotation L',...
    'Arm flexion R','Arm adduction R','Arm rotation R',...
    'Elbow flexion L','Elbow flexion R'};
muscleNames = {'Glut med 1','Glut med 2','Glut med 3',...
        'Glut min 1','Glut min 2','Glut min 3','Semimem',...
        'Semiten','bifemlh','Bic fem sh','Sar','Add long',...
        'Add brev','Add mag 1','Add mag 2','Add mag 3','TFL',...
        'Pect','Grac','Glut max 1','Glut max 2','Glut max 3',......
        'Iliacus','Psoas','Quad fem','Gem','Peri',...
        'Rect fem','Vas med','Vas int','Vas lat','Med gas',...
        'Lat gas','Soleus','Tib post','Flex dig','Flex hal',...
        'Tib ant','Per brev','Per long','Per tert','Ext dig',...
        'Ext hal','Ercspn','Intobl','Extobl'};
GRF_str = {'Fore-aft','Vertical','Lateral'};

%% Load reference data
pathReferenceData = [pathrepo,'/ExperimentalData'];
load([pathReferenceData,'/ExperimentalData.mat'],'ExperimentalData');

%% Common settings for plots
label_fontsize  = 16;
sup_fontsize  = 24;
line_linewidth  = 3;
col = hsv(length(ww));

%% Plot joint angles
Qref = ExperimentalData.Q;
idx_Qs = [1,2,3,10,11,12,14,16,18,19,20,21,25,26,27,29];
NumTicks = 2;
figure()
for i = 1:length(idx_Qs)
    subplot(4,4,i)
    p = gobjects(1,length(ww));
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Qs_opt(ww(k)).m,1)-1):100;
        p(k) = plot(x,Qs_opt(ww(k)).m(:,idx_Qs(i)),...
            'color',col(k,:),'linewidth',line_linewidth);
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
    hold on
    fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
    alpha(.25);
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);        
    title(joints_tit{idx_Qs(i)},'Fontsize',label_fontsize);
    % Y-axis
    if i == 1 || i == 5 || i == 9 ||i == 13
        ylabel('Angle (°)','Fontsize',label_fontsize);
    end
    % X-axis
    L = get(gca,'XLim');    
    if i > 12
        set(gca,'XTick',linspace(L(1),L(2),NumTicks))
        xlabel('Gait cycle (%)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end
end 
l = legend(p,legend_case);
set(l,'Fontsize',label_fontsize)
sp = sgtitle('Joint angles');
set(sp,'Fontsize',sup_fontsize);   

%% Plot joint velocities
if ~showMainPlots
figure()
for i = 1:length(idx_Qs)
    subplot(4,4,i)
    p = gobjects(1,length(ww));
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Qdots_opt(ww(k)).m,1)-1):100;
        p(k) = plot(x,Qdots_opt(ww(k)).m(:,idx_Qs(i)),...
            'color',col(k,:),'linewidth',line_linewidth);
        hold on;    
    end
    % Experimental data
    idx_jref = strcmp(Qref.(subject).Qdots.colheaders,joints_ref{i});
    meanPlusSTD = Qref.(subject).Qdots.mean(:,idx_jref) + 2*Qref.(subject).Qdots.std(:,idx_jref);
    meanMinusSTD = Qref.(subject).Qdots.mean(:,idx_jref) - 2*Qref.(subject).Qdots.std(:,idx_jref);          
    stepQ = (size(Qdots_opt(ww(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
    intervalQ = 1:stepQ:size(Qdots_opt(ww(k)).m,1);
    sampleQ = 1:size(Qdots_opt(ww(k)).m,1);
    meanPlusSTD = interp1(intervalQ,meanPlusSTD,sampleQ);
    meanMinusSTD = interp1(intervalQ,meanMinusSTD,sampleQ);
    hold on
    fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
    alpha(.25);
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);        
    title(joints_tit{idx_Qs(i)},'Fontsize',label_fontsize);
    % Y-axis
    if i == 1 || i == 5 || i == 9 ||i == 13
        ylabel('Velocity (°/s)','Fontsize',label_fontsize);
    end
    % X-axis
    L = get(gca,'XLim');    
    if i > 12
        set(gca,'XTick',linspace(L(1),L(2),NumTicks))
        xlabel('Gait cycle (%)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end
end 
l = legend(p,legend_case);
set(l,'Fontsize',label_fontsize)
sp = sgtitle('Joint velocities');
set(sp,'Fontsize',sup_fontsize);  

end

%% Plot ground reaction forces
GRFref = ExperimentalData.GRFs;
ylim_GRF = ([-50,50;0 200;-25 25]);
NumTicks_GRF = 2;
figure()
for i = 1:length(GRF_str)
    subplot(1,3,i)
    % Simulation results
    p = gobjects(1,length(ww));
    for k = 1:length(ww)
        x = 1:(100-1)/(size(GRFs_opt(ww(k)).m,1)-1):100;
        p(k) = plot(x,GRFs_opt(ww(k)).m(:,i),'color',...
            col(k,:),'linewidth',line_linewidth);
        hold on;         
    end    
    % Experimental data
    meanPlusSTD = GRFref.(subject).mean(:,i) + 2*GRFref.(subject).std(:,i);    
    meanMinusSTD = GRFref.(subject).mean(:,i) - 2*GRFref.(subject).std(:,i);   
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
    title(GRF_str{i},'Fontsize',label_fontsize);
    % Y-axis    
    ylim([ylim_GRF(i,1) ylim_GRF(i,2)]);
    L = get(gca,'YLim');
    if i == 1 || i ==3
        set(gca,'YTick',[L(1),0,L(2)]);    
    else
        set(gca,'YTick',linspace(L(1),L(2),NumTicks_GRF));      
    end
    if i == 1
        ylabel('Force (%BW)','Fontsize',label_fontsize);
    end
    % X-axis
    L = get(gca,'XLim');
    set(gca,'XTick',linspace(L(1),L(2),NumTicks_GRF))
    xlabel('Gait cycle (%)','Fontsize',label_fontsize); 
end
l = legend(p,legend_case);
set(l,'Fontsize',16)
sp = sgtitle('Ground reaction forces');
set(sp,'Fontsize',sup_fontsize);

%% Plot joint kinetics
IDref = ExperimentalData.Torques;
figure()
for i = 1:length(idx_Qs)-3
    subplot(4,4,i)
    % Simulation results
    p = gobjects(1,length(ww));
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Ts_opt(ww(k)).m,1)-1):100;
        p(k) = plot(x,Ts_opt(ww(k)).m(:,idx_Qs(i+3))*body_mass,...
            'color',col(k,:),'linewidth',line_linewidth);
        hold on;    
    end
    % Experimental data
    idx_jref = strcmp(IDref.(subject).colheaders,joints_ref{i+3});
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
    title(joints_tit{idx_Qs(i+3)},'Fontsize',label_fontsize);
    % Y-axis
    if i == 1 || i == 5 || i == 9 ||i == 13
        ylabel('Torque (Nm)','Fontsize',label_fontsize);
    end
    % X-axis    
    L = get(gca,'XLim');
    NumTicks = 2;
    if i > 9 
        set(gca,'XTick',linspace(L(1),L(2),NumTicks))
        xlabel('Gait cycle (%)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end
end 
l = legend(p,legend_case);
set(l,'Fontsize',16)
sp = sgtitle('Joint torques');
set(sp,'Fontsize',sup_fontsize);

%% Plot joint powers
Pref = ExperimentalData.Powers;
figure()
for i = 1:length(idx_Qs)-3
    subplot(4,4,i)
    % Simulation results
    p = gobjects(1,length(ww));
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Qdots_opt(ww(k)).m,1)-1):100;
        p(k) = plot(x,Qdots_opt(ww(k)).m(:,idx_Qs(i+3)).*pi/180.*...
            Ts_opt(ww(k)).m(:,idx_Qs(i+3))*body_mass,...
            'color',col(k,:),'linewidth',line_linewidth);
        hold on;    
    end
    % Experimental data
    idx_jref = strcmp(Pref.(subject).colheaders,joints_ref{i+3});
    meanPlusSTD = Pref.(subject).mean(:,idx_jref) + 2*Pref.(subject).std(:,idx_jref);
    meanMinusSTD = Pref.(subject).mean(:,idx_jref) - 2*Pref.(subject).std(:,idx_jref);          
    stepQ = (size(Qdots_opt(ww(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
    intervalQ = 1:stepQ:size(Qdots_opt(ww(k)).m,1);
    sampleQ = 1:size(Qdots_opt(ww(k)).m,1);
    meanPlusSTD = interp1(intervalQ,meanPlusSTD,sampleQ);
    meanMinusSTD = interp1(intervalQ,meanMinusSTD,sampleQ);
    hold on
    fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
    alpha(.25);
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);   
    title(joints_tit{idx_Qs(i+3)},'Fontsize',label_fontsize);
    % Y-axis
    if i == 1 || i == 5 || i == 9 ||i == 13
        ylabel('Power (W)','Fontsize',label_fontsize);
    end
    % X-axis    
    L = get(gca,'XLim');
    NumTicks = 2;
    if i > 9 
        set(gca,'XTick',linspace(L(1),L(2),NumTicks))
        xlabel('Gait cycle (%)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end
end 
l = legend(p,legend_case);
set(l,'Fontsize',16)
sp = sgtitle('Joint powers');
set(sp,'Fontsize',sup_fontsize);  
   
%% Plot muscle activations (right)
EMGref = ExperimentalData.EMG;
% Selecting muscles
EMGchannel = [9,9,9,...
    99,99,99,5,...
    5,1,1,99,13,...
    99,99,99,99,14,...
    99,99,99,99,99,...
    99,99,99,99,99,...
    10,8,7,7,12,...
    4,6,99,99,99,...
    2,11,3,99,99,...
    99,99,99,99];
EMGcol = ones(1,length(muscleNames));
EMGcol(EMGchannel==99)=0;
EMGref.(subject).allnorm = ...
    NaN(2*N,size(EMGref.(subject).all,2),size(EMGref.(subject).all,3));
% Plot   
NMuscle = size(Acts_opt(ww(k)).m,2);
figure()
for i = 1:size(Acts_opt(ww(k)).m,2)/2
    subplot(7,7,i)
    p = gobjects(1,length(ww));   
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Acts_opt(ww(k)).m,1)-1):100;
        p(k) = plot(x,Acts_opt(ww(k)).m(:,i+NMuscle/2),'color',...
            col(k,:),'linewidth',line_linewidth);
        hold on;
    end
    if EMGcol(i)
        % Normalize peak EMG to peak muscle activation
        a_peak = max(Acts_opt(ww(k)).m(:,i+NMuscle/2));
        emg_peak = zeros(1,size(EMGref.(subject).all,3));
        for j = 1:size(EMGref.(subject).all,3)
            emg_peak(j) = nanmax(EMGref.(subject).all(:,EMGchannel(i),j),[],1);
        end
        norm_f = a_peak./emg_peak;           
        tempp(:,:) = EMGref.(subject).all(:,EMGchannel(i),:);
        intervalInterp = 1:(size(tempp,1)-1)/(2*N-1):size(tempp,1);
        temp = interp1(1:size(tempp,1),tempp,intervalInterp);        
        temp = temp.*repmat(norm_f,2*N,1);
        EMGref.(subject).allnorm(:,EMGchannel(i),:) = temp;
        EMGref.(subject).meannorm = nanmean(EMGref.(subject).allnorm,3);
        EMGref.(subject).stdnorm = nanstd(EMGref.(subject).allnorm,[],3);        
        meanPlusSTD = EMGref.(subject).meannorm(:,EMGchannel(i)) + 2*EMGref.(subject).stdnorm(:,EMGchannel(i));
        meanMinusSTD = EMGref.(subject).meannorm(:,EMGchannel(i)) - 2*EMGref.(subject).stdnorm(:,EMGchannel(i));
        stepa = (size(Acts_opt(ww(k)).m,1)-1)/(size(meanMinusSTD,1)-1);
        intervala = 1:stepa:size(Acts_opt(ww(k)).m,1);
        samplea = 1:size(Acts_opt(ww(k)).m,1);
        meanPlusSTD = interp1(intervala,meanPlusSTD,samplea);
        meanMinusSTD = interp1(intervala,meanMinusSTD,samplea);     
        hold on
        fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
        alpha(.25);            
    end
    % Plot settings
    set(gca,'Fontsize',label_fontsize)
    title(muscleNames{i},'Fontsize',label_fontsize);    
    % X-axis
    L = get(gca,'XLim');
    NumTicks = 3;
    if i > 39
        set(gca,'XTick',linspace(L(1),L(2),NumTicks))
        xlabel('Gait cycle (%)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end
    % Y-axis
    ylim([0,1]);
    NumTicks = 2;
    LY = get(gca,'YLim');
    set(gca,'YTick',linspace(LY(1),LY(2),NumTicks))
    if i == 1 || i == 8 || i == 15 || i == 22 || i == 29 || i == 36 || i == 43
        ylabel('(-)','Fontsize',20);
    end    
end
l = legend(p,legend_case);
set(l,'Fontsize',label_fontsize)
sp = sgtitle('Muscle activations: right');
set(sp,'Fontsize',sup_fontsize);

%% Plot muscle activations (left)
if ~showMainPlots
figure()
for i = 1:size(Acts_opt(ww(k)).m,2)/2
    subplot(7,7,i)
    p = gobjects(1,length(ww));
    NMuscle = size(Acts_opt(ww(k)).m,2);
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Acts_opt(ww(k)).m,1)-1):100;
        p(k) = plot(x,Acts_opt(ww(k)).m(:,i),'color',col(k,:),...
            'linewidth',line_linewidth);
        hold on;
    end
    % Plot settings
    set(gca,'Fontsize',label_fontsize)
    title(muscleNames{i},'Fontsize',label_fontsize);    
    % X-axis
    L = get(gca,'XLim');
    NumTicks = 3;
    if i > 39
        set(gca,'XTick',linspace(L(1),L(2),NumTicks))
        xlabel('Gait cycle (%)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end
    % Y-axis
    ylim([0,1]);
    NumTicks = 2;
    LY = get(gca,'YLim');
    set(gca,'YTick',linspace(LY(1),LY(2),NumTicks))
    if i == 1 || i == 8 || i == 15 || i == 22 || i == 29 || i == 36 || i == 43
        ylabel('(-)','Fontsize',20);
    end    
end
l = legend(p,legend_case);
set(l,'Fontsize',label_fontsize)
sp = sgtitle('Muscle activations: left');
set(sp,'Fontsize',sup_fontsize);

end

%% Plot COT / stride length / step width / CPU time / Optimal cost
figure()
subplot(2,3,1)
p = gobjects(1,length(ww));
tempp = zeros(length(ww),1);
for k = 1:length(ww)
    p(k) = scatter(k,COT_opt(ww(k)).m,...
        40,col(k,:),'filled');
    tempp(k) = COT_opt(ww(k)).m;
    hold on; 
end
maxy = ceil(max(tempp));
ylim([2 maxy]);
set(gca,'Fontsize',label_fontsize);
title('COT','Fontsize',label_fontsize);
ylabel('(J kg-1 m-1)','Fontsize',label_fontsize);

subplot(2,3,2)
p = gobjects(1,length(ww));
for k = 1:length(ww)
    p(k) = scatter(k,StrideLength_opt(ww(k)).m,...
        40,col(k,:),'filled');
    tempp(k) = StrideLength_opt(ww(k)).m;
    hold on;    
end
maxy = ceil(max(tempp));
ylim([1 maxy]);
set(gca,'Fontsize',label_fontsize);
title('Stride length','Fontsize',label_fontsize);
ylabel('(m)','Fontsize',label_fontsize);

subplot(2,3,3)
p = gobjects(1,length(ww));
for k = 1:length(ww)
    p(k) = scatter(k,mean(StepWidth_opt(ww(k)).m),...
        40,col(k,:),'filled');
    hold on;    
end
ylim([0 0.2]);
set(gca,'Fontsize',label_fontsize);
title('Step width','Fontsize',label_fontsize);
ylabel('(m)','Fontsize',label_fontsize);

subplot(2,3,4)
p = gobjects(1,length(ww));
for k = 1:length(ww)
    p(k) = scatter(k,(CPU_IPOPT(ww(k)).m+CPU_NLP(ww(k)).m),...
        40,col(k,:),'filled');
    hold on;    
end
set(gca,'Fontsize',label_fontsize);
title('Computational time','Fontsize',label_fontsize);
ylabel('(s)','Fontsize',label_fontsize);

subplot(2,3,5)
p = gobjects(1,length(ww));
for k = 1:length(ww)
    p(k) = scatter(k,(Cost(ww(k)).m),...
        40,col(k,:),'filled');
    hold on;    
end
set(gca,'Fontsize',label_fontsize);
title('Optimal cost','Fontsize',label_fontsize);
ylabel('(-)','Fontsize',label_fontsize);
l = legend(p,legend_case);
set(l,'Fontsize',label_fontsize)
