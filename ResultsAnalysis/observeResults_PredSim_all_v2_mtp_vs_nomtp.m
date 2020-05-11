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
ww_mtp  = [12]; 
ww_nomtp  = [22]; 
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
% user choice
showMainPlots = 0; % 0 to also see joint velocities and left m activations

%% Load results (with mtp)
ww = ww_mtp;
settings = getSettings_predSim_all_mtp();
% Pre-allocation structures
Qs_opt              = struct('m',[]);
Qdots_opt           = struct('m',[]);
Acts_opt            = struct('m',[]);
GRFs_opt            = struct('m',[]);
Ts_opt              = struct('m',[]);
COT_opt             = struct('m',[]);
StrideLength_opt    = struct('m',[]);
StepWidth_opt       = struct('m',[]);
legend_case         = cell(1,length(ww));
% Loop over cases
for k = 1:length(ww)
    predSim_data_all_v2_mtp;
    legend_case{k} = ['Case: ',num2str(ww(k))];
end
Qs_opt_mtp = Qs_opt;
Qdots_opt_mtp = Qdots_opt;
Acts_opt_mtp = Acts_opt;
Ts_opt_mtp = Ts_opt;
GRFs_opt_mtp = GRFs_opt;
COT_opt_mtp = COT_opt;
StrideLength_opt_mtp = StrideLength_opt;
StepWidth_opt_mtp = StepWidth_opt;

%% Load results (without mtp)
ww = ww_nomtp;
settings = getSettings_predSim_all();
% Pre-allocation structures
Qs_opt              = struct('m',[]);
Qdots_opt           = struct('m',[]);
Acts_opt            = struct('m',[]);
GRFs_opt            = struct('m',[]);
Ts_opt              = struct('m',[]);
COT_opt             = struct('m',[]);
StrideLength_opt    = struct('m',[]);
StepWidth_opt       = struct('m',[]);
legend_case         = cell(1,length(ww));
% Loop over cases
for k = 1:length(ww)
    predSim_data_all_v2;
    legend_case{k} = ['Case: ',num2str(ww(k))];
end
Qs_opt_nomtp = Qs_opt;
Qdots_opt_nomtp = Qdots_opt;
Acts_opt_nomtp = Acts_opt;
Ts_opt_nomtp = Ts_opt;
GRFs_opt_nomtp = GRFs_opt;
COT_opt_nomtp = COT_opt;
StrideLength_opt_nomtp = StrideLength_opt;
StepWidth_opt_nomtp = StepWidth_opt;

%% Helper names
joints_ref = {'pelvis_tilt','pelvis_list','pelvis_rotation',...
    'hip_flexion','hip_adduction','hip_rotation',...
    'knee_angle','ankle_angle','subtalar_angle','mtp_angle',...
    'lumbar_extension','lumbar_bending','lumbar_rotation',...
    'arm_flex','arm_add','arm_rot','elbow_flex'};
joints_tit = {'Pelvis tilt','Pelvis list','Pelvis rotation','Pelvis tx',...
    'Pelvis ty','Pelvis tz','Hip flexion L','Hip adduction L',...
    'Hip rotation L','Hip flexion R','Hip adduction R','Hip rotation R',...
    'Knee L','Knee R','Ankle L','Ankle R',...
    'Subtalar L','Subtalar R','MTP L','MTP R',...
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
col = hsv(length(ww_mtp) + length(ww_nomtp));

%% Plot joint angles
Qref = ExperimentalData.Q;
idx_Qs_mtp       = [1,2,3,10,11,12,14,16,18,20,21,22,23,27,28,29,31];
idx_Qs_nomtp = [1,2,3,10,11,12,14,16,18,NaN,19,20,21,25,26,27,29];
NumTicks = 2;
figure()
for i = 1:length(idx_Qs_mtp)
    subplot(3,6,i)
    p = gobjects(1,length(ww_mtp) + length(ww_nomtp));
    % Simulation results (with mtp)
    for k = 1:length(ww_mtp)
        x = 1:(100-1)/(size(Qs_opt_mtp(ww_mtp(k)).m,1)-1):100;
        p(k) = plot(x,Qs_opt_mtp(ww_mtp(k)).m(:,idx_Qs_mtp(i)),...
            'color',col(k,:),'linewidth',line_linewidth);
        hold on;    
    end
    if ~(strcmp(joints_ref{i},'mtp_angle'))
        for k = 1:length(ww_nomtp)
            x = 1:(100-1)/(size(Qs_opt_nomtp(ww_nomtp(k)).m,1)-1):100;
            p(k+length(ww_mtp)) = plot(x,Qs_opt_nomtp(ww_nomtp(k)).m(:,idx_Qs_nomtp(i)),...
                'color',col(k+length(ww_mtp),:),'linewidth',line_linewidth);
            hold on;    
        end
    end
    % Experimental data
    if ~(strcmp(joints_ref{i},'mtp_angle'))
        idx_jref = strcmp(Qref.(subject).Qs.colheaders,joints_ref{i});
        meanPlusSTD = Qref.(subject).Qs.mean(:,idx_jref) + 2*Qref.(subject).Qs.std(:,idx_jref);
        meanMinusSTD = Qref.(subject).Qs.mean(:,idx_jref) - 2*Qref.(subject).Qs.std(:,idx_jref);          
        stepQ = (size(Qs_opt_mtp(ww_mtp(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
        intervalQ = 1:stepQ:size(Qs_opt_mtp(ww_mtp(k)).m,1);
        sampleQ = 1:size(Qs_opt_mtp(ww_mtp(k)).m,1);
        meanPlusSTD = interp1(intervalQ,meanPlusSTD,sampleQ);
        meanMinusSTD = interp1(intervalQ,meanMinusSTD,sampleQ);
        hold on
        fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
        alpha(.25);
    end
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);        
    title(joints_tit{idx_Qs_mtp(i)},'Fontsize',label_fontsize);
    % Y-axis
    if i == 1 || i == 7 ||i == 13
        ylabel('Angle (°)','Fontsize',label_fontsize);
    end
    % X-axis
    L = get(gca,'XLim');    
    if i > 11
        set(gca,'XTick',linspace(L(1),L(2),NumTicks))
        xlabel('Gait cycle (%)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end
end 
l = legend(p,'With mtp joint','Without mtp joint');
set(l,'Fontsize',label_fontsize)
sp = suptitle('Joint angles');
set(sp,'Fontsize',sup_fontsize);   

%% Plot joint velocities
if ~showMainPlots
figure()
for i = 1:length(idx_Qs_mtp)
    subplot(3,6,i)
    p = gobjects(1,length(ww_mtp) + length(ww_nomtp));
    % Simulation results
    for k = 1:length(ww_mtp)
        x = 1:(100-1)/(size(Qdots_opt_mtp(ww_mtp(k)).m,1)-1):100;
        p(k) = plot(x,Qdots_opt_mtp(ww_mtp(k)).m(:,idx_Qs_mtp(i)),...
            'color',col(k,:),'linewidth',line_linewidth);
        hold on;    
    end
    if ~(strcmp(joints_ref{i},'mtp_angle'))
        for k = 1:length(ww_nomtp)
            x = 1:(100-1)/(size(Qdots_opt_nomtp(ww_nomtp(k)).m,1)-1):100;
            p(k+length(ww_mtp)) = plot(x,Qdots_opt_nomtp(ww_nomtp(k)).m(:,idx_Qs_nomtp(i)),...
                'color',col(k+length(ww_mtp),:),'linewidth',line_linewidth);
            hold on;    
        end
    end
    % Experimental data
    if ~(strcmp(joints_ref{i},'mtp_angle'))
        idx_jref = strcmp(Qref.(subject).Qdots.colheaders,joints_ref{i});
        meanPlusSTD = Qref.(subject).Qdots.mean(:,idx_jref) + 2*Qref.(subject).Qdots.std(:,idx_jref);
        meanMinusSTD = Qref.(subject).Qdots.mean(:,idx_jref) - 2*Qref.(subject).Qdots.std(:,idx_jref);          
        stepQ = (size(Qdots_opt_mtp(ww_mtp(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
        intervalQ = 1:stepQ:size(Qdots_opt_mtp(ww_mtp(k)).m,1);
        sampleQ = 1:size(Qdots_opt_mtp(ww_mtp(k)).m,1);
        meanPlusSTD = interp1(intervalQ,meanPlusSTD,sampleQ);
        meanMinusSTD = interp1(intervalQ,meanMinusSTD,sampleQ);
        hold on
        fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
        alpha(.25);
    end
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);        
    title(joints_tit{idx_Qs_mtp(i)},'Fontsize',label_fontsize);
    % Y-axis
    if i == 1 || i == 7 ||i == 13
        ylabel('Velocity (°/s)','Fontsize',label_fontsize);
    end
    % X-axis
    L = get(gca,'XLim');    
    if i > 11
        set(gca,'XTick',linspace(L(1),L(2),NumTicks))
        xlabel('Gait cycle (%)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end
end 
l = legend(p,'With mtp joint','Without mtp joint');
set(l,'Fontsize',label_fontsize)
sp = suptitle('Joint velocities');
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
    p = gobjects(1,length(ww_mtp) + length(ww_nomtp));
    for k = 1:length(ww_mtp)
        x = 1:(100-1)/(size(GRFs_opt_mtp(ww_mtp(k)).m,1)-1):100;
        p(k) = plot(x,GRFs_opt_mtp(ww_mtp(k)).m(:,i),'color',...
            col(k,:),'linewidth',line_linewidth);
        hold on;         
    end    
    for k = 1:length(ww_nomtp)
        x = 1:(100-1)/(size(GRFs_opt_nomtp(ww_nomtp(k)).m,1)-1):100;
        p(k+length(ww_mtp)) = plot(x,GRFs_opt_nomtp(ww_nomtp(k)).m(:,i),'color',...
            col(k+length(ww_mtp),:),'linewidth',line_linewidth);
        hold on;         
    end    
    
    % Experimental data
    meanPlusSTD = GRFref.(subject).mean(:,i) + 2*GRFref.(subject).std(:,i);    
    meanMinusSTD = GRFref.(subject).mean(:,i) - 2*GRFref.(subject).std(:,i);   
    stepGRF = (size(GRFs_opt_mtp(ww_mtp(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
    intervalGRF = 1:stepGRF:size(GRFs_opt_mtp(ww_mtp(k)).m,1);
    sampleGRF = 1:size(GRFs_opt_mtp(ww_mtp(k)).m,1);
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
l = legend(p,'With mtp joint','Without mtp joint');
set(l,'Fontsize',16)
sp = suptitle('Ground reaction forces');
set(sp,'Fontsize',sup_fontsize);

%% Plot joint kinetics
IDref = ExperimentalData.Torques;
figure()
for i = 1:length(idx_Qs_mtp)-3
    subplot(3,6,i)
    % Simulation results
    p = gobjects(1,length(ww_mtp) + length(ww_nomtp));
    for k = 1:length(ww_mtp)
        x = 1:(100-1)/(size(Ts_opt_mtp(ww_mtp(k)).m,1)-1):100;
        p(k) = plot(x,Ts_opt_mtp(ww_mtp(k)).m(:,idx_Qs_mtp(i+3))*body_mass,...
            'color',col(k,:),'linewidth',line_linewidth);
        hold on;    
    end
    if ~(strcmp(joints_ref{i+3},'mtp_angle'))
        for k = 1:length(ww_nomtp)
            x = 1:(100-1)/(size(Ts_opt_nomtp(ww_nomtp(k)).m,1)-1):100;
            p(k+length(ww_mtp)) = plot(x,Ts_opt_nomtp(ww_nomtp(k)).m(:,idx_Qs_nomtp(i+3))*body_mass,...
                'color',col(k+length(ww_mtp),:),'linewidth',line_linewidth);
            hold on;    
        end
    end
    % Experimental data
    if ~(strcmp(joints_ref{i+3},'mtp_angle'))
        idx_jref = strcmp(IDref.(subject).colheaders,joints_ref{i+3});
        meanPlusSTD = IDref.(subject).mean(:,idx_jref) + 2*IDref.(subject).std(:,idx_jref);
        meanMinusSTD = IDref.(subject).mean(:,idx_jref) - 2*IDref.(subject).std(:,idx_jref);  
        stepID = (size(Ts_opt_mtp(ww_mtp(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
        intervalID = 1:stepID:size(Ts_opt_mtp(ww_mtp(k)).m,1);
        sampleID = 1:size(Ts_opt_mtp(ww_mtp(k)).m,1);
        meanPlusSTD = interp1(intervalID,meanPlusSTD,sampleID);
        meanMinusSTD = interp1(intervalID,meanMinusSTD,sampleID); 
        hold on
        fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
        alpha(.25);
    end
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);   
    title(joints_tit{idx_Qs_mtp(i+3)},'Fontsize',label_fontsize);
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
l = legend(p,'With mtp joint','Without mtp joint');
set(l,'Fontsize',16)
sp = suptitle('Joint torques');
set(sp,'Fontsize',sup_fontsize);

%% Plot joint powers
Pref = ExperimentalData.Powers;
figure()
for i = 1:length(idx_Qs_mtp)-3
    subplot(4,4,i)
    % Simulation results
    p = gobjects(1,length(ww_mtp) + length(ww_nomtp));
    for k = 1:length(ww_mtp)
        x = 1:(100-1)/(size(Qdots_opt_mtp(ww_mtp(k)).m,1)-1):100;
        p(k) = plot(x,Qdots_opt_mtp(ww_mtp(k)).m(:,idx_Qs_mtp(i+3)).*pi/180.*...
            Ts_opt_mtp(ww_mtp(k)).m(:,idx_Qs_mtp(i+3))*body_mass,...
            'color',col(k,:),'linewidth',line_linewidth);
        hold on;    
    end
    if ~(strcmp(joints_ref{i+3},'mtp_angle'))
        for k = 1:length(ww_nomtp)
            x = 1:(100-1)/(size(Qdots_opt_nomtp(ww_nomtp(k)).m,1)-1):100;
            p(k+length(ww_nomtp)) = plot(x,Qdots_opt_nomtp(ww_nomtp(k)).m(:,idx_Qs_nomtp(i+3)).*pi/180.*...
                Ts_opt_nomtp(ww_nomtp(k)).m(:,idx_Qs_nomtp(i+3))*body_mass,...
                'color',col(k+length(ww_nomtp),:),'linewidth',line_linewidth);
            hold on;    
        end
    end
    % Experimental data
    if ~(strcmp(joints_ref{i+3},'mtp_angle'))
        idx_jref = strcmp(Pref.(subject).colheaders,joints_ref{i+3});
        meanPlusSTD = Pref.(subject).mean(:,idx_jref) + 2*Pref.(subject).std(:,idx_jref);
        meanMinusSTD = Pref.(subject).mean(:,idx_jref) - 2*Pref.(subject).std(:,idx_jref);          
        stepQ = (size(Qdots_opt_mtp(ww_mtp(k)).m,1)-1)/(size(meanPlusSTD,1)-1);
        intervalQ = 1:stepQ:size(Qdots_opt_mtp(ww_mtp(k)).m,1);
        sampleQ = 1:size(Qdots_opt_mtp(ww_mtp(k)).m,1);
        meanPlusSTD = interp1(intervalQ,meanPlusSTD,sampleQ);
        meanMinusSTD = interp1(intervalQ,meanMinusSTD,sampleQ);
        hold on
        fill([x fliplr(x)],[meanPlusSTD fliplr(meanMinusSTD)],'k');
        alpha(.25);
    end
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);   
    title(joints_tit{idx_Qs_mtp(i+3)},'Fontsize',label_fontsize);
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
l = legend(p,'With mtp joint','Without mtp joint');
set(l,'Fontsize',16)
sp = suptitle('Joint powers');
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
NMuscle = size(Acts_opt_mtp(ww_mtp(k)).m,2);
figure()
for i = 1:size(Acts_opt_mtp(ww_mtp(k)).m,2)/2
    subplot(7,7,i)
    p = gobjects(1,length(ww_mtp) + length(ww_nomtp)); 
    % Simulation results
    for k = 1:length(ww_mtp)
        x = 1:(100-1)/(size(Acts_opt_mtp(ww_mtp(k)).m,1)-1):100;
        p(k) = plot(x,Acts_opt_mtp(ww_mtp(k)).m(:,i+NMuscle/2),'color',...
            col(k,:),'linewidth',line_linewidth);
        hold on;
    end
    for k = 1:length(ww_nomtp)
        x = 1:(100-1)/(size(Acts_opt_nomtp(ww_nomtp(k)).m,1)-1):100;
        p(k+length(ww_nomtp)) = plot(x,Acts_opt_nomtp(ww_nomtp(k)).m(:,i+NMuscle/2),'color',...
            col(k+length(ww_nomtp),:),'linewidth',line_linewidth);
        hold on;
    end
    if EMGcol(i)
        % Normalize peak EMG to peak muscle activation
        a_peak = max(Acts_opt_mtp(ww_mtp(k)).m(:,i+NMuscle/2));
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
        stepa = (size(Acts_opt_mtp(ww_mtp(k)).m,1)-1)/(size(meanMinusSTD,1)-1);
        intervala = 1:stepa:size(Acts_opt_mtp(ww_mtp(k)).m,1);
        samplea = 1:size(Acts_opt_mtp(ww_mtp(k)).m,1);
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
l = legend(p,'With mtp joint','Without mtp joint');
set(l,'Fontsize',label_fontsize)
sp = suptitle('Muscle activations: right');
set(sp,'Fontsize',sup_fontsize);

%% Plot muscle activations (left)
if ~showMainPlots
figure()
for i = 1:size(Acts_opt_mtp(ww_mtp(k)).m,2)/2
    subplot(7,7,i)
    p = gobjects(1,length(ww_mtp));
    NMuscle = size(Acts_opt_mtp(ww_mtp(k)).m,2);
    for k = 1:length(ww_mtp)
        x = 1:(100-1)/(size(Acts_opt_mtp(ww_mtp(k)).m,1)-1):100;
        p(k) = plot(x,Acts_opt_mtp(ww_mtp(k)).m(:,i),'color',col(k,:),...
            'linewidth',line_linewidth);
        hold on;
    end
    for k = 1:length(ww_nomtp)
        x = 1:(100-1)/(size(Acts_opt_nomtp(ww_nomtp(k)).m,1)-1):100;
        p(k+length(ww_mtp)) = plot(x,Acts_opt_nomtp(ww_nomtp(k)).m(:,i),...
            'color',col(k+length(ww_mtp),:), 'linewidth',line_linewidth);
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
l = legend(p,'With mtp joint','Without mtp joint');
set(l,'Fontsize',label_fontsize)
sp = suptitle('Muscle activations: left');
set(sp,'Fontsize',sup_fontsize);

end

%% Plot COT / stride length / step width / CPU time / Optimal cost
figure()
subplot(1,3,1)
p = gobjects(1,length(ww_mtp) + length(ww_nomtp));
tempp = zeros(length(ww_mtp),1);
for k = 1:length(ww_mtp)
    p(k) = scatter(k,COT_opt_mtp(ww_mtp(k)).m,...
        40,col(k,:),'filled');
    tempp(k) = COT_opt_mtp(ww_mtp(k)).m;
    hold on; 
end
for k = 1:length(ww_nomtp)
    p(k+length(ww_mtp)) = scatter(k,COT_opt_nomtp(ww_nomtp(k)).m,...
        40,col(k+length(ww_mtp),:),'filled');
    tempp(k+length(ww_mtp)) = COT_opt_nomtp(ww_nomtp(k)).m;
    hold on; 
end
maxy = ceil(max(tempp));
ylim([2 maxy]);
set(gca,'Fontsize',label_fontsize);
title('COT','Fontsize',label_fontsize);
ylabel('(J kg-1 m-1)','Fontsize',label_fontsize);

subplot(1,3,2)
p = gobjects(1,length(ww_mtp));
for k = 1:length(ww_mtp)
    p(k) = scatter(k,StrideLength_opt_mtp(ww_mtp(k)).m,...
        40,col(k,:),'filled');
    tempp(k) = StrideLength_opt_mtp(ww_mtp(k)).m;
    hold on;    
end
for k = 1:length(ww_nomtp)
    p(k+length(ww_nomtp)) = scatter(k,StrideLength_opt_nomtp(ww_nomtp(k)).m,...
        40,col(k+length(ww_nomtp),:),'filled');
    tempp(k+length(ww_nomtp)) = StrideLength_opt_nomtp(ww_nomtp(k)).m;
    hold on;    
end
maxy = ceil(max(tempp));
ylim([1 maxy]);
set(gca,'Fontsize',label_fontsize);
title('Stride length','Fontsize',label_fontsize);
ylabel('(m)','Fontsize',label_fontsize);

subplot(1,3,3)
p = gobjects(1,length(ww_mtp));
for k = 1:length(ww_mtp)
    p(k) = scatter(k,mean(StepWidth_opt_mtp(ww_mtp(k)).m),...
        40,col(k,:),'filled');
    hold on;    
end
for k = 1:length(ww_nomtp)
    p(k+length(ww_nomtp)) = scatter(k,mean(StepWidth_opt_nomtp(ww_nomtp(k)).m),...
        40,col(k+length(ww_nomtp),:),'filled');
    hold on;    
end
ylim([0 0.2]);
set(gca,'Fontsize',label_fontsize);
title('Step width','Fontsize',label_fontsize);
ylabel('(m)','Fontsize',label_fontsize);
l = legend(p,'With mtp joint','Without mtp joint');
set(l,'Fontsize',label_fontsize)

