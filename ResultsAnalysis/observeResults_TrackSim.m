% This script reproduces Fig S4
% Author: Antoine Falisse
% Date: 1/7/2019

clear all
close all
clc

%% Settings
% Selected trial
% 1: nominal cost function
ww  = 2; 
% Fixed settings
subject = 'subject1';
body_mass = 62;
body_weight = body_mass*9.81;
settings = [10,1,1,10,1,25,50,50,4;
    10,1,1,10,1,50,100,50,4];
setup.derivatives = 'AD';
showtrackplotsonly = 0;
showlegend = 0;
writeModel = 1;

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
ParamsCM_opt    = struct('m',[]);
ParamsCM_gen    = struct('m',[]);
dev_cm          = struct('m',[]);
% Loop over cases
for k = 1:length(ww) 
    % Load results
    pathmain = pwd;
    [pathRepo,~,~] = fileparts(pathmain);
    pathresults = [pathRepo,'/Results'];
    ocp_path = 'TrackSim_mtp';
    load([pathresults,'/',ocp_path,'/Results_tracking.mat']);    
    % Unstructure data   
    Qs_opt(ww(k)).m = Results_tracking.Qs_opt;
    Ts_opt(ww(k)).m = Results_tracking.Ts_opt;
    GRFs_opt(ww(k)).m = Results_tracking.GRFs_opt;
    GRMs_opt(ww(k)).m = Results_tracking.GRMs_opt; 
    ParamsCM_opt(ww(k)).m = Results_tracking.ParamsCM_opt;
    ParamsCM_gen(ww(k)).m = Results_tracking.ParamsCM_gen;
    dev_cm(ww(k)).m  = Results_tracking.dev_cm;
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
pos_Qs = 1:31;
ylim_Qs = [-10,10;-10,10;-10,10;-1,1;0.75,1;-0.5,0;-50,50;-20,20;-20,20;...
    -50,50;-20,20;-20,20;-80,0;-80,0;-30,30;-30,30;-20,20;-20,20;-10,60;...
    -10,60;-10,0;-15,15;-15,15;-15,15;-30,0;0,30;-15,15;-30,0;0,30;0,50;0,50];
idx_Qs = 1:31; 
NumTicks_Qs = 2;
RefData_str_tit = {'pelvis-tilt','pelvis-list','pelvis-rotation',...
    'pelvis-tx','pelvis-ty','pelvis-tz','Hip flex L',...
    'Hip add L','Hip rot L','Hip flex R',...
    'Hip add R','Hip rot R','Knee L','Knee R',...
    'Ankle L','Ankle R','Subtalar L',...
    'Subtalar R','MTP L','MTP R','Lumbar extension','Lumbar bending',...
    'Lumbar rotation','Arm flex L','Arm add L','Arm rot L',...
    'Arm flex R','Arm add R','Arm rot R',...
    'Elbow flex L','Elbow flex R'};
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
pos_GRF = [1:3,7:9];
ylim_GRF = [-50,50;0 150;-25 25;-50,50;0 150;-25 25];
NumTicks_GRF = 2;
temp = GRFs_toTrack(ww(1)).m(:,[3,6]);
temp(temp<0) = 0;
GRFs_toTrack(ww(1)).m(:,[3,6])= temp;
GRF_str = {'Fore-aft R','Vertical R','Lateral R',...
    'Fore-aft L','Vertical L','Lateral L'};
figure(2)
for i = 1:length(GRF_str)
    subplot(2,6,pos_GRF(i))
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
pos_GRM = [4:6,10:12];
ylim_GRM = [-20,200;-50 50;-250 250;-20,200;-50 50;-250 250];
figure(2)
for i = 1:length(GRF_str)
    subplot(2,6,pos_GRM(i))
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
idx_Qdots = 1:31; 
ylim_Qdots = [-5,5;-5,5;-5,5;-5,5;-5,5;-5,5;...
    -50,50;-100,100;-20,20;-50,50;-100,100;-20,20;...
    -60,60;-60,60;-100,10;-100,10;-20,20;-20,20;-20,20;-20,20;...
    -20,20;-20,20;-20,20;
    -20,20;-20,20;-20,20;-20,20;-20,20;-20,20;-20,20;-20,20];
pos_Qdots = 1:31;
figure(3)
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

%% Parameters
for k = 1:length(ww)
    figure()
    if length(ParamsCM_opt(ww(k)).m) == 12
        subplot(1,2,1)
        p(k) = scatter(1:8,ParamsCM_opt(ww(k)).m(1:8),'filled','MarkerFaceColor',color_all(2,:),'linewidth',2); hold on
        scatter(1:8,paramsCM_gen(ww(k)).m(1:8),'k','linewidth',2);
%         params_bounds_upper = (setup(ww(k)).m.setup.bounds.params.upper - ...
%             setup(ww(k)).m.setup.scaling.params.r)./setup(ww(k)).m.setup.scaling.params.v;
%         params_bounds_lower = (setup(ww(k)).m.setup.bounds.params.lower - ...
%             setup(ww(k)).m.setup.scaling.params.r)./setup(ww(k)).m.setup.scaling.params.v;
%         scatter(1:8,params_bounds_upper(1:8),'b','linewidth',2);
%         scatter(1:8,params_bounds_lower(1:8),'r','linewidth',2);
        l = legend('Optimal','Generic');
        set(l,'Fontsize',16);
        subplot(1,2,2)
        scatter(1:4,ParamsCM_opt(ww(k)).m(9:12),'filled','MarkerFaceColor',color_all(2,:),'linewidth',2); hold on
        scatter(1:4,paramsCM_gen(ww(k)).m(9:12),'k','linewidth',2);        
%         scatter(1:4,params_bounds_upper(9:12),'b','linewidth',2);
%         scatter(1:4,params_bounds_lower(9:12),'r','linewidth',2);
        l = legend('Optimal','Generic');
        set(l,'Fontsize',16);
        if writeModel
            pathVariousFunctions = [pathRepo,'\VariousFunctions'];
            addpath(genpath(pathVariousFunctions));    
            pathData = [pathRepo,'\OpenSimModel\',subject];    
            ModelSetup = xml_read([pathData,'\',subject,'_mtp_ContactsAsForces.osim']);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).location(1) = ParamsCM_opt(ww(k)).m(1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).location(3) = ParamsCM_opt(ww(k)).m(2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).location(1) = ParamsCM_opt(ww(k)).m(3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).location(3) = ParamsCM_opt(ww(k)).m(4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(1) = ParamsCM_opt(ww(k)).m(5);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(3) = ParamsCM_opt(ww(k)).m(6);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).location(1) = ParamsCM_opt(ww(k)).m(7);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).location(3) = ParamsCM_opt(ww(k)).m(8);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).location(1) = ParamsCM_opt(ww(k)).m(1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).location(3) = -ParamsCM_opt(ww(k)).m(2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(6).location(1) = ParamsCM_opt(ww(k)).m(3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(6).location(3) = -ParamsCM_opt(ww(k)).m(4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(1) = ParamsCM_opt(ww(k)).m(5);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(3) = -ParamsCM_opt(ww(k)).m(6);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).location(1) = ParamsCM_opt(ww(k)).m(7);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).location(3) = -ParamsCM_opt(ww(k)).m(8);
            PathNewModel = [pathresults,'\',ocp_path,'\',subject,'_scaled_contact_opt_',num2str(settings(ww(k),6)),'cs_','devL',num2str(settings(ww(k),7)),'_devS',num2str(settings(ww(k),8)),'_',nametrial.id,'.osim'];
            xml_writeOSIM(PathNewModel,ModelSetup,'OpenSimDocument');
        end     
    elseif length(ParamsCM_opt(ww(k)).m) == 18
        nCS = 6;
        subplot(1,2,1)
        p(k) = scatter(1:12,ParamsCM_opt(ww(k)).m(1:12),'filled','MarkerFaceColor',color_all(2,:),'linewidth',2); hold on
        scatter(1:12,ParamsCM_gen(ww(k)).m(1:12),'k','linewidth',2);        
        params_bounds_upper = ParamsCM_gen(ww(k)).m(1:12) + dev_cm(ww(k)).m.loc/1000;
        params_bounds_lower = ParamsCM_gen(ww(k)).m(1:12) - dev_cm(ww(k)).m.loc/1000;
        scatter(1:12,params_bounds_upper,'b','linewidth',2);
        scatter(1:12,params_bounds_lower,'r','linewidth',2);
        l = legend('Optimal','Generic');
        set(l,'Fontsize',16);
        subplot(1,2,2)
        scatter(1:6,ParamsCM_opt(ww(k)).m(13:18),'filled','MarkerFaceColor',color_all(2,:),'linewidth',2); hold on
        scatter(1:6,ParamsCM_gen(ww(k)).m(13:18),'k','linewidth',2);    
        radii = 0.032*ones(1,6);
        params_bounds_upper = radii + dev_cm(ww(k)).m.rad/100*radii; 
        params_bounds_lower = radii - dev_cm(ww(k)).m.rad/100*radii;
        scatter(1:6,params_bounds_upper,'b','linewidth',2);
        scatter(1:6,params_bounds_lower,'r','linewidth',2);
        l = legend('Optimal','Generic');
        set(l,'Fontsize',16);
        if writeModel
            pathVariousFunctions = [pathRepo,'\VariousFunctions'];
            addpath(genpath(pathVariousFunctions));    
            pathData = [pathRepo,'\OpenSimModel\',subject];    
            ModelSetup = xml_read([pathData,'\',subject,'_mtp_ContactsAsForces.osim']);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).location(1) = ParamsCM_opt(ww(k)).m(1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).location(3) = ParamsCM_opt(ww(k)).m(2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).location(1) = ParamsCM_opt(ww(k)).m(3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).location(3) = ParamsCM_opt(ww(k)).m(4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(1) = ParamsCM_opt(ww(k)).m(5);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(3) = ParamsCM_opt(ww(k)).m(6);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).location(1) = ParamsCM_opt(ww(k)).m(7);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).location(3) = ParamsCM_opt(ww(k)).m(8);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).location(1) = ParamsCM_opt(ww(k)).m(9);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).location(3) = ParamsCM_opt(ww(k)).m(10);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(6).location(1) = ParamsCM_opt(ww(k)).m(11);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(6).location(3) = ParamsCM_opt(ww(k)).m(12);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(1) = ParamsCM_opt(ww(k)).m(1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(3) = -ParamsCM_opt(ww(k)).m(2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).location(1) = ParamsCM_opt(ww(k)).m(3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).location(3) = -ParamsCM_opt(ww(k)).m(4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).location(1) = ParamsCM_opt(ww(k)).m(5);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).location(3) = -ParamsCM_opt(ww(k)).m(6);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).location(1) = ParamsCM_opt(ww(k)).m(7);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).location(3) = -ParamsCM_opt(ww(k)).m(8);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).location(1) = ParamsCM_opt(ww(k)).m(9);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).location(3) = -ParamsCM_opt(ww(k)).m(10);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(12).location(1) = ParamsCM_opt(ww(k)).m(11);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(12).location(3) = -ParamsCM_opt(ww(k)).m(12);            
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).radius = ParamsCM_opt(ww(k)).m(13);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).radius = ParamsCM_opt(ww(k)).m(14);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).radius = ParamsCM_opt(ww(k)).m(15);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).radius = ParamsCM_opt(ww(k)).m(16);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).radius = ParamsCM_opt(ww(k)).m(17);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(6).radius = ParamsCM_opt(ww(k)).m(18);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).radius = ParamsCM_opt(ww(k)).m(13);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).radius = ParamsCM_opt(ww(k)).m(14);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).radius = ParamsCM_opt(ww(k)).m(15);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).radius = ParamsCM_opt(ww(k)).m(16);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).radius = ParamsCM_opt(ww(k)).m(17);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(12).radius = ParamsCM_opt(ww(k)).m(18);                   
            PathNewModel = [pathresults,'\',ocp_path,'\',subject,'_mtp_contact_opt_','devL',num2str(settings(ww(k),7)),'_devS',num2str(settings(ww(k),8)),'.osim'];
            xml_writeOSIM(PathNewModel,ModelSetup,'OpenSimDocument');
        end
    end
end

