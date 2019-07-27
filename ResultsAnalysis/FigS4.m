% This script reproduces Fig S4
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
body_mass = 62;
body_weight = body_mass*9.81;
settings = [10,1,1,10,1,25,50,50,4];
setup.derivatives = 'AD';
showtrackplotsonly = 0;
showlegend = 0;

%% Load results
% Pre-allocation structures
Qs_opt          = struct('m',[]);
Ts_opt          = struct('m',[]);
GRFs_opt        = struct('m',[]);
GRMs_opt        = struct('m',[]);
Qs_toTrack      = struct('m',[]);
Ts_toTrack      = struct('m',[]);
GRFs_toTrack    = struct('m',[]);
GRMs_toTrack    = struct('m',[]);
% Loop over cases
for k = 1:length(ww) 
    % Load results
    pathmain = pwd;
    [pathRepo,~,~] = fileparts(pathmain);
    pathresults = [pathRepo,'/Results'];
    load([pathresults,'/TrackSim/Results_tracking.mat']);    
    % Unstructure data   
    Qs_opt(ww(k)).m = Results_tracking.Qs_opt;
    Ts_opt(ww(k)).m = Results_tracking.Ts_opt;
    GRFs_opt(ww(k)).m = Results_tracking.GRFs_opt;
    GRMs_opt(ww(k)).m = Results_tracking.GRMs_opt; 
    Qs_toTrack(ww(k)).m = Results_tracking.Qs_toTrack;
    Ts_toTrack(ww(k)).m = Results_tracking.Ts_toTrack;
    GRFs_toTrack(ww(k)).m = Results_tracking.GRFs_toTrack;
    GRMs_toTrack(ww(k)).m = Results_tracking.GRMs_toTrack; 
end

%% Common settings for plots
label_fontsize  = 14;
line_linewidth  = 3;
% Colors
color_all(1,:) = [0,0,0];
color_all(2,:) = [219,50,54]/255; % Red

%% Plot joint kinematics
pos_Qs = [1:3,7:9,13:15,19:21];
ylim_Qs = [-50,50;-20,20;-20,20;-50,50;-20,20;-20,20;-80,0;-30,30;-20,20;-80,0;-30,30;-20,20];
idx_Qs = [10,11,12,7,8,9,14,16,18,13,15,17]; 
NumTicks_Qs = 2;
RefData_str_tit = {'pelvis-tilt','pelvis-list','pelvis-rotation',...
    'pelvis-tx','pelvis-ty','pelvis-tz','Hip flex L',...
    'Hip add L','Hip rot L','Hip flex R',...
    'Hip add R','Hip rot R','Knee L','Knee R',...
    'Ankle L','Ankle R','Subtalar L',...
    'Subtalar R','lumbar-extension','lumbar-bending',...
    'lumbar-rotation','arm-flex-l','arm-add-l','arm-rot-l',...
    'arm-flex-r','arm-add-r','arm-rot-r',...
    'elbow-flex-l','elbow-flex-r'};
figure()
for i = 1:length(idx_Qs)
    subplot(6,6,pos_Qs(i))
    % Experimental data
    Qs_toTrack_deg = Qs_toTrack(ww(k)).m;
    Qs_toTrack_deg(:,[2:4,8:end]) = Qs_toTrack_deg(:,[2:4,8:end])*180/pi;    
    plot(Qs_toTrack_deg(:,1),Qs_toTrack_deg(:,idx_Qs(i)+1),...
        'color',color_all(1,:),'linewidth',line_linewidth);  
    hold on
    % Simulation results
    for k = 1:length(ww)
        plot(Qs_toTrack(ww(k)).m(:,1),Qs_opt(ww(k)).m(:,idx_Qs(i)),...
            'color',color_all(2,:),'linestyle',':','linewidth',line_linewidth);
        hold on;          
    end    
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);    
    title(RefData_str_tit{idx_Qs(i)},'Fontsize',label_fontsize);  
    % Y-axis
    ylim([ylim_Qs(i,1) ylim_Qs(i,2)]);
    L = get(gca,'YLim');
    set(gca,'YTick',linspace(L(1),L(2),NumTicks_Qs));       
    if i == 1 || i == 4 || i == 7 || i == 10
        ylabel('(°)','Fontsize',label_fontsize);
    end      
    % X-axis
    xlim([Qs_toTrack_deg(1,1),Qs_toTrack_deg(end,1)])
    set(gca,'XTick',[]);
    box off;
end 

%% Plot ground reaction forces
pos_GRF = [25:27,31:33];
ylim_GRF = [-50,50;0 150;-25 25;-50,50;0 150;-25 25];
NumTicks_GRF = 2;
temp = GRFs_toTrack(ww(1)).m(:,[3,6]);
temp(temp<0) = 0;
GRFs_toTrack(ww(1)).m(:,[3,6])= temp;
GRF_str = {'Fore-aft R','Vertical R','Lateral R',...
    'Fore-aft L','Vertical L','Lateral L'};
for i = 1:length(GRF_str)
    subplot(6,6,pos_GRF(i))
    % Experimental data
    plot(GRFs_toTrack(ww(k)).m(:,1),GRFs_toTrack(ww(1)).m(:,i+1)./(body_weight/100),...
        'color',color_all(1,:),'linewidth',line_linewidth);
    hold on;
    % Simulation results
    for k = 1:length(ww)
        plot(GRFs_toTrack(ww(k)).m(:,1),GRFs_opt(ww(k)).m(:,i)./(body_weight/100),...
            'color',color_all(2,:),'linestyle',':','linewidth',line_linewidth);
        hold on;    
    end
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);    
    title(GRF_str{i},'Fontsize',label_fontsize);  
    % Y-axis
    ylim([ylim_GRF(i,1) ylim_GRF(i,2)]);
    L = get(gca,'YLim');
    set(gca,'YTick',linspace(L(1),L(2),NumTicks_GRF));       
    if i == 1 || i == 4 
        ylabel('(%BW)','Fontsize',label_fontsize);
    end      
    % X-axis
    xlim([Qs_toTrack_deg(1,1),Qs_toTrack_deg(end,1)])
    L = get(gca,'XLim');
    if i == 4 || i == 5 || i == 6
        set(gca,'XTick',linspace(L(1),L(2),NumTicks_GRF))
        xlabel('Time (s)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end
    box off;   
end

%% Plot ground reaction moments expressed wrt ground frame origin
pos_GRM = [28:30,34:36];
ylim_GRM = [-20,200;-50 50;-250 250;-20,200;-50 50;-250 250];
for i = 1:length(GRF_str)
    subplot(6,6,pos_GRM(i))
    % Experimental data
    plot(GRMs_toTrack(ww(k)).m(:,1),GRMs_toTrack(ww(k)).m(:,i+1),...
        'color',color_all(1,:),'linewidth',line_linewidth);
    hold on;
    % Simulation results
    for k = 1:length(ww)
        plot(GRMs_toTrack(ww(k)).m(:,1),GRMs_opt(ww(k)).m(:,i),...
            'color',color_all(2,:),'linestyle',':','linewidth',line_linewidth);
        hold on;    
    end
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);    
    title(GRF_str{i},'Fontsize',label_fontsize);  
    % Y-axis
    ylim([ylim_GRM(i,1) ylim_GRM(i,2)]);
    L = get(gca,'YLim');
    set(gca,'YTick',linspace(L(1),L(2),NumTicks_GRF));       
    if i == 1 || i == 4 
        ylabel('(Nm)','Fontsize',label_fontsize);
    end      
    % X-axis
    xlim([Qs_toTrack_deg(1,1),Qs_toTrack_deg(end,1)])
    L = get(gca,'XLim');
    if i == 4 || i == 5 || i == 6
        set(gca,'XTick',linspace(L(1),L(2),NumTicks_GRF))
        xlabel('Time (s)','Fontsize',label_fontsize);
    else
        set(gca,'XTick',[]);
    end
    box off;    
end

%% Plot joint kinetics
idx_Qdots = [10,11,12,7,8,9,14,16,18,13,15,17]; 
ylim_Qdots = [-50,50;-100,100;-20,20;-50,50;-100,100;-20,20;-60,60;...
    -100,10;-20,20;-60,60;-100,10;-20,20];
pos_Qdots = [4:6,10:12,16:18,22:24];
for i = 1:length(idx_Qdots)
    subplot(6,6,pos_Qdots(i))
    % Experimental data
    plot(Ts_toTrack(ww(k)).m(:,1),Ts_toTrack(ww(k)).m(:,idx_Qdots(i)+1),...
        'color',color_all(1,:),'linewidth',line_linewidth);
    hold on
    % Simulation results
    for k = 1:length(ww)
        plot(Ts_toTrack(ww(k)).m(:,1),Ts_opt(ww(k)).m(:,idx_Qdots(i)),...
            'color',color_all(2,:),'linestyle',':','linewidth',line_linewidth);
        hold on;            
    end
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);    
    title(RefData_str_tit{idx_Qs(i)},'Fontsize',label_fontsize);  
    % Y-axis
    ylim([ylim_Qdots(i,1) ylim_Qdots(i,2)]);
    L = get(gca,'YLim');
    set(gca,'YTick',linspace(L(1),L(2),NumTicks_Qs));       
    if i == 1 || i == 4 || i == 7 || i == 10
        ylabel('(Nm)','Fontsize',label_fontsize);
    end      
    % X-axis
    xlim([Qs_toTrack_deg(1,1),Qs_toTrack_deg(end,1)])
    set(gca,'XTick',[]);
    box off;
end    
