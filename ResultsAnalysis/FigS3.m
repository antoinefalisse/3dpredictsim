% This script reproduces Fig S3
% Author: Antoine Falisse
% Date: 6/3/2019

clear all
close all
clc

%% Settings
% Selected trials
% 194:  low co-contraction (lower bounds on muscle activations: 0.1)
% 196:  medium co-contraction (lower bounds on muscle activations: 0.15)
% 197:  high co-contraction (lower bounds on muscle activations: 0.2)
% 1:    nominal settings (bounds on muscle activations: 0.05))
ww  = [194,196,197,1]; 
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
    % Extract also trunk sway measures, obtained post-processing
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
color_all(1,:)  = [72,133,237]/255;     % Blue
color_all(2,:)  = [244,194,13]/255;     % Yellow
color_all(3,:)  = [219,50,54]/255;      % Red
color_all(4,:)  = [0,0,0];              % Black
color_exp       = [168,168,168]/255;    % Grey

pos_Qs      = 1:4;
pos_T       = 7:10;
pos_P       = 13:16;
pos_GRF     = 19:21;
pos_trunk   = 22;
pos_COT     = 1;
pos_width   = 2;
pos_length  = 3;
pos_Qs2      = 7:10;
pos_T2       = 13:16;
pos_P2       = 19:22;
pos_act      = 1:10;

NSubPlots = [4,6];

%% Names for plots
joints_ref = {'hip_flexion','hip_adduction',...
    'knee_angle','ankle_angle'};
joints_tit = {'Hip flexion','Hip adduction',...
    'Knee','Ankle'};
muscleNames = {'glut-med1','glut-med2','glut-med3',...
    'glut-min1','glut-min2','glut-min3','semimem',...
    'semiten','bifemlh','bifemsh','sar','add-long',...
    'add-brev','add-mag1','add-mag2','add-mag3','tfl',...
    'pect','grac','glut-max1','glut-max2','glut-max3',......
    'iliacus','psoas','quad-fem','gem','peri',...
    'rect-fem','vas-med','vas-int','vas-lat','med-gas',...
    'lat-gas','soleus','tib-post','flex-dig','flex-hal',...
    'tib-ant','per-brev','per-long','per-tert','ext-dig',...
    'ext-hal','ercspn','intobl','extobl'}; 
muscleNames_sel = {'glut-med2','glut-min1','semiten','bifemsh',...
    'vas-lat','rect-fem','lat-gas','med-gas','soleus','tib-ant'};
muscleNames_tit = {'Gluteus med','Gluteus min','Semiten',...
    'Biceps fem sh',...
    'Vastus lat','Rectus fem','Gastroc lat','Gastroc med','Soleus','Tibialis ant'};
GRF_str = {'Fore-aft','Vertical','Lateral'};

%% Plot joint kinematics
Qref = ExperimentalData.Q;
ylim_Qs = [-50,50;-20,20;-80,20;-40,40];
idx_Qs = [10,11,14,16];
figure()
for i = 1:length(idx_Qs)
    subplot(NSubPlots(1),NSubPlots(2),pos_Qs(i))
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Qs_opt(ww(k)).m,1)-1):100;
        plot(x,Qs_opt(ww(k)).m(:,idx_Qs(i)),'color',color_all(k,:),'linewidth',line_linewidth);
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
ylim_T = [-100,100;-100,100;-100,100;-120,120];
for i = 1:length(idx_Qs)
    subplot(NSubPlots(1),NSubPlots(2),pos_T(i))
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Ts_opt(ww(k)).m,1)-1):100;
        plot(x,Ts_opt(ww(k)).m(:,idx_Qs(i))*body_mass,'color',color_all(k,:),'linewidth',line_linewidth);
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
    set(gca,'XTick',[]);
    box off;
end 

%% Plot ground reaction forces
GRFref = ExperimentalData.GRFs;
ylim_GRF = ([-50,50;0 150;-25 25]);
NumTicks_GRF = 2;
for i = 1:length(GRF_str)
    subplot(NSubPlots(1),NSubPlots(2),pos_GRF(i))
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(GRFs_opt(ww(k)).m,1)-1):100;        
        plot(x,GRFs_opt(ww(k)).m(:,i),'color',...
                color_all(k,:),'linewidth',line_linewidth);  
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
    title(GRF_str{(i)},'Fontsize',label_fontsize);  
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
    xlabel('(%GC)','Fontsize',label_fontsize);
    box off;
end   

%% Plot trunk sway
TrunkSwayref = ExperimentalData.TrunkSway;
ylim_trunk = [-10,10];
x = 1:100;
NumTicks_trunkX = 2;
subplot(NSubPlots(1),NSubPlots(2),pos_trunk)
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
set(gca,'YTick',[L(1),0,L(2)]);  
ylabel('Angle (°)','Fontsize',label_fontsize);
% X-axis
L = get(gca,'XLim');
set(gca,'XTick',linspace(L(1),L(2),NumTicks_trunkX))
xlabel('(%GC)','Fontsize',label_fontsize);
box off;

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
COT_order_mean = COT_order([end,1:end-1]);
COT_order_mean = [COTref.mean,COT_order_mean];
COT_order_mean = [COT_order_mean;zeros(1,length(COT_order_mean))];
COT_order_std = zeros(2,length(COT_order_mean));
COT_order_std(1,1) = COTref.std;
% Plot
ylim_COT = [0,8];
xlim_COT = [0.4 1.6];
NumTicks_COT = 2;
figure()
subplot(NSubPlots(1),NSubPlots(2),pos_COT)
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
color_COT = color_all([end,1:end-1],:);
for k = 1:length(ww)
    set(h_COT(k+1),'FaceColor',color_COT(k,:));
end
box off;

%% Plot step widths
StepWidthref = ExperimentalData.StepWidth;
% Post-processing for barplot
stride_width_all.std = [StepWidthref.(subject).std,zeros(1,length(ww))];
stride_width_all.mean = [StepWidthref.(subject).mean,zeros(1,length(ww))];
for k = 1:length(ww)
    stride_width_all.mean(1,k+1) = StepWidth_opt(ww(k)).m;
end
stride_width_all.std = [stride_width_all.std;zeros(1,length(stride_width_all.std))];
stride_width_all.mean = [stride_width_all.mean;zeros(1,length(stride_width_all.mean))];
stride_width_all.mean(1,2:end) = stride_width_all.mean(1,[end,2:end-1]);
% Plot
ylim_width = [0,0.3];
xlim_width = [0.4 1.6];
NumTicks_width = 2;
subplot(NSubPlots(1),NSubPlots(2),pos_width)
h_width = barwitherr(stride_width_all.std,stride_width_all.mean);
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
ylabel('(m)','Fontsize',label_fontsize);
% Colors
set(h_width(1),'FaceColor',color_exp);
color_width = color_all([end,1:end-1],:);
for k = 1:length(ww)
    set(h_width(k+1),'FaceColor',color_width(k,:));
end
box off;

%% Plot stride lengths
StrideLengthref = ExperimentalData.StrideLength;
% Post-processing for barplot
stride_length_all.std = [StrideLengthref.(subject).std,zeros(1,length(ww))];
stride_length_all.mean = [StrideLengthref.(subject).mean,zeros(1,length(ww))];
for k = 1:length(ww)
    stride_length_all.mean(1,k+1) = mean(StrideLength_opt(ww(k)).m);
end
stride_length_all.std = [stride_length_all.std;zeros(1,length(stride_length_all.std))];
stride_length_all.mean = [stride_length_all.mean;zeros(1,length(stride_length_all.mean))];
stride_length_all.mean(1,2:end) = stride_length_all.mean(1,[end,2:end-1]);
% Plot
ylim_length = [0,2];
xlim_length = [0.4 1.6];
NumTicks_length = 2;
subplot(NSubPlots(1),NSubPlots(2),pos_length)
h_length = barwitherr(stride_length_all.std,stride_length_all.mean);
% Plot settings 
set(gca,'Fontsize',label_fontsize);
title('Stride length','Fontsize',label_fontsize);
% X-axis
xlim([xlim_length(1,1) xlim_length(1,2)]);
set(gca,'XTick',[]);
% Y-axis
ylim([ylim_length(1,1) ylim_length(1,2)]);
L = get(gca,'YLim');
set(gca,'YTick',linspace(L(1),L(2),NumTicks_length)); 
% Colors
set(h_length(1),'FaceColor',color_exp);
color_length = color_all([end,1:end-1],:);
for k = 1:length(ww)
    set(h_length(k+1),'FaceColor',color_length(k,:));
end
box off;

%% Plot muscle activations  
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
NMuscle = size(Acts_opt(ww(k)).m,2);
idx_m = zeros(1,length(muscleNames_sel));
for i = 1:length(muscleNames_sel)
    idx_m(i) = find(strcmp(muscleNames,muscleNames_sel{i}));
end
% Plot
ylim_act = [0 1]; 
NumTicks_act = 2;
figure()
for i = 1:length(muscleNames_sel)
    subplot(NSubPlots(1),NSubPlots(2),pos_act(i))    
    % Simulation results
    for k = 1:length(ww)
        x = 1:(100-1)/(size(Acts_opt(ww(k)).m,1)-1):100;
        plot(x,Acts_opt(ww(k)).m(:,idx_m(i)+NMuscle/2),'color',...
            color_all(k,:),'linewidth',3);
        hold on;
    end    
    % Experimental data
    if EMGcol(idx_m(i))
        k = length(ww);
        % Normalize peak EMG to peak muscle activation
        a_peak = max(Acts_opt(ww(k)).m(:,idx_m(i)+NMuscle/2));
        emg_peak = zeros(1,size(EMGref.(subject).all,3));
        for j = 1:size(EMGref.(subject).all,3)
            emg_peak(j) = nanmax(EMGref.(subject).all(:,EMGchannel(idx_m(i)),j),[],1);
        end
        norm_f = a_peak./emg_peak;          
        tempp(:,:) = EMGref.(subject).all(:,EMGchannel(idx_m(i)),:);
        intervalInterp = 1:(size(tempp,1)-1)/(2*N-1):size(tempp,1);
        temp = interp1(1:size(tempp,1),tempp,intervalInterp);        
        temp = temp.*repmat(norm_f,2*N,1);
        EMGref.(subject).allnorm(:,EMGchannel(idx_m(i)),:) = temp;
        EMGref.(subject).meannorm = nanmean(EMGref.(subject).allnorm,3);
        EMGref.(subject).stdnorm = nanstd(EMGref.(subject).allnorm,[],3);        
        meanPlusSTD = EMGref.(subject).meannorm(:,EMGchannel(idx_m(i))) + 2*EMGref.(subject).stdnorm(:,EMGchannel(idx_m(i)));
        meanMinusSTD = EMGref.(subject).meannorm(:,EMGchannel(idx_m(i))) - 2*EMGref.(subject).stdnorm(:,EMGchannel(idx_m(i)));
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
    set(gca,'Fontsize',label_fontsize);
    title(muscleNames_tit{i},'Fontsize',label_fontsize);
    % Y-axis
    ylim([ylim_act(1,1) ylim_act(1,2)]);
    L = get(gca,'YLim');
    set(gca,'YTick',linspace(L(1),L(2),NumTicks_act));       
    if i == 1 || i == 7
        ylabel('Activation (-)','Fontsize',label_fontsize);
    end  
    % X-axis
    if i >= 7 
        L = get(gca,'XLim');
        set(gca,'XTick',linspace(L(1),L(2),NumTicks_act))
        xlabel('(%GC)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end   
    box off
end
