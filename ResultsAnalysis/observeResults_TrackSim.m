% This script produces figures for the tracking simulations
% Author: Antoine Falisse
% Date: 1/7/2019

clear all
close all
clc

%% Settings
% Selected trial
% 1: nominal cost function
idx_ww  = 25; 
mtp_pin = 1; % the mtp joint is a pin rather than a custom joint
% Fixed settings
subject = 'subject1';
body_mass = 62;
body_weight = body_mass*9.81;
pathmain = pwd;
[pathRepo,~,~] = fileparts(pathmain);
pathOCP = [pathRepo,'/OCP'];
addpath(genpath(pathOCP));
trackSim_mtp_settings_all

setup.derivatives = 'AD';
showtrackplotsonly = 0;
showlegend = 0;
writeModel = 0;

%% Load results
% Pre-allocation structures
Qs_opt          = struct('ww',[]);
Ts_opt          = struct('ww',[]);
aMTP_opt        = struct('ww',[]);
passT_opt       = struct('ww',[]);
GRFs_opt        = struct('ww',[]);
GRMs_opt        = struct('ww',[]);
Qs_toTrack      = struct('ww',[]);
Ts_toTrack      = struct('ww',[]);
GRFs_toTrack    = struct('ww',[]);
GRMs_toTrack    = struct('ww',[]);
ParamsCM_opt    = struct('ww',[]);
ParamsCM_gen    = struct('ww',[]);
dev_cm          = struct('ww',[]);
bounds          = struct('ww',[]);
trials          = struct('ww',[]);
% Loop over cases
pathmain = pwd;
[pathRepo,~,~] = fileparts(pathmain);
pathresults = [pathRepo,'/Results'];
ocp_path = 'TrackSim_mtp';
load([pathresults,'/',ocp_path,'/Results_tracking.mat']);
for www = 1:length(idx_ww)
    ww = idx_ww(www);
    trials(ww).ww = settings_trials(ww).ww;
    % Load results   
    for p = 1:length(trials(ww).ww) 
        Qs_opt(ww).ww(p).p = Results_tracking(ww).ww(p).Qs_opt;
        Ts_opt(ww).ww(p).p = Results_tracking(ww).ww(p).Ts_opt;
        GRFs_opt(ww).ww(p).p = Results_tracking(ww).ww(p).GRFs_opt;
        GRMs_opt(ww).ww(p).p = Results_tracking(ww).ww(p).GRMs_opt;
        aMTP_opt(ww).ww(p).p = Results_tracking(ww).ww(p).aMTP; 
        passT_opt(ww).ww(p).p = Results_tracking(ww).ww(p).passT; 
        Qs_toTrack(ww).ww(p).p = Results_tracking(ww).ww(p).Qs_toTrack;
        Ts_toTrack(ww).ww(p).p = Results_tracking(ww).ww(p).Ts_toTrack;
        GRFs_toTrack(ww).ww(p).p = Results_tracking(ww).ww(p).GRFs_toTrack;
        GRMs_toTrack(ww).ww(p).p = Results_tracking(ww).ww(p).GRMs_toTrack; 
        ParamsCM_opt(ww).ww(p).p = Results_tracking(ww).ww(p).ParamsCM_opt;
        bounds(ww).ww(p).p = Results_tracking(ww).ww(p).bounds;
        ParamsCM_gen(ww).ww(p).p = Results_tracking(ww).ww(p).ParamsCM_gen;
        dev_cm(ww).ww(p).p  = Results_tracking(ww).ww(p).dev_cm; 
    end       
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
for p = 1:length(trials(ww).ww)  
    figure()
    for i = 1:length(idx_Qs)
        subplot(6,6,pos_Qs(i))        
        for k = 1:length(idx_ww)
            % Experimental data
            ww = idx_ww(k);
            Qs_toTrack_deg = Qs_toTrack(ww).ww(p).p;
            Qs_toTrack_deg(:,[2:4,8:end]) = Qs_toTrack_deg(:,[2:4,8:end])*180/pi;    
            plot(Qs_toTrack_deg(:,1),Qs_toTrack_deg(:,idx_Qs(i)+1),...
                'color',color_all(1,:),'linewidth',line_linewidth);  
            hold on
            % Simulation results        
            plot(Qs_toTrack(ww).ww(p).p(:,1),Qs_opt(ww).ww(p).p(:,idx_Qs(i)),...
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
end

%% Plot ground reaction forces & moments expressed wrt ground frame origin
pos_GRF = [1:3,7:9];
ylim_GRF = [-50,50;0 150;-25 25;-50,50;0 150;-25 25];
NumTicks_GRF = 2;
GRF_str = {'Fore-aft R','Vertical R','Lateral R',...
    'Fore-aft L','Vertical L','Lateral L'};
pos_GRM = [4:6,10:12];
ylim_GRM = [-250,150;-50 50;-150 200;-150,200;-50 50;-250 300];
for p = 1:length(trials(ww).ww)      
    figure()
    for i = 1:length(GRF_str)
        subplot(2,6,pos_GRF(i))
        for k = 1:length(idx_ww)
            ww = idx_ww(k);
            Qs_toTrack_deg = Qs_toTrack(ww).ww(p).p;
            temp = GRFs_toTrack(ww).ww(p).p(:,[3,6]);
            temp(temp<0) = 0;
            GRFs_toTrack(ww).ww(p).p(:,[3,6])= temp;
            % Experimental data
            plot(GRFs_toTrack(ww).ww(p).p(:,1),GRFs_toTrack(ww).ww(p).p(:,i+1)./(body_weight/100),...
                'color',color_all(1,:),'linewidth',line_linewidth);
            hold on;
            % Simulation results            
            plot(GRFs_toTrack(ww).ww(p).p(:,1),GRFs_opt(ww).ww(p).p(:,i)./(body_weight/100),...
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
    
    for i = 1:length(GRF_str)
        subplot(2,6,pos_GRM(i))
        for k = 1:length(idx_ww)
            ww = idx_ww(k);
            Qs_toTrack_deg = Qs_toTrack(ww).ww(p).p;
            % Experimental data
            plot(GRMs_toTrack(ww).ww(p).p(:,1),GRMs_toTrack(ww).ww(p).p(:,i+1),...
                'color',color_all(1,:),'linewidth',line_linewidth);
            hold on;
            % Simulation results    
            plot(GRMs_toTrack(ww).ww(p).p(:,1),GRMs_opt(ww).ww(p).p(:,i),...
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
end

%% Plot joint kinetics
idx_Qdots = 1:31; 
ylim_Qdots = [-5,5;-5,5;-5,5;-5,5;-5,5;-5,5;...
    -50,50;-100,100;-20,20;-50,50;-100,100;-20,20;...
    -60,60;-60,60;-100,10;-100,10;-20,20;-20,20;-20,20;-20,20;...
    -20,20;-20,20;-20,20;
    -20,20;-20,20;-20,20;-20,20;-20,20;-20,20;-20,20;-20,20];
pos_Qdots = 1:31;
for p = 1:length(trials(ww).ww)  
    figure()
    for i = 1:length(idx_Qdots)
        subplot(6,6,pos_Qdots(i))
        for k = 1:length(idx_ww)
            ww = idx_ww(k);
            Qs_toTrack_deg = Qs_toTrack(ww).ww(p).p;
            % Experimental data
            plot(Ts_toTrack(ww).ww(p).p(:,1),Ts_toTrack(ww).ww(p).p(:,idx_Qdots(i)+1),...
                'color',color_all(1,:),'linewidth',line_linewidth);
            hold on
            % Simulation results    
            plot(Ts_toTrack(ww).ww(p).p(:,1),Ts_opt(ww).ww(p).p(:,idx_Qdots(i)),...
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
end

%% MTP passive vs active torques
idx_mtp = 19:20;
pos_mtp = 1:2;
for p = 1:length(trials(ww).ww)  
    figure()
    for i = 1:length(idx_mtp)
        subplot(1,2,pos_mtp(i))
        for k = 1:length(idx_ww)
            ww = idx_ww(k);
            Qs_toTrack_deg = Qs_toTrack(ww).ww(p).p;
            % Simulation results    
            plot(Ts_toTrack(ww).ww(p).p(:,1),Ts_opt(ww).ww(p).p(:,idx_mtp(i)),...
                'color','k','linewidth',line_linewidth); 
            hold on
            % Passive torques
            plot(Ts_toTrack(ww).ww(p).p(:,1),passT_opt(ww).ww(p).p(:,idx_mtp(i)-6),...
                'color',color_all(2,:),'linewidth',line_linewidth); 
            % Active torques
            plot(Ts_toTrack(ww).ww(p).p(:,1),aMTP_opt(ww).ww(p).p(:,i)*100,...
                'color','b','linewidth',line_linewidth);
            l = legend('ID','Passive (stiffness + damping)','Active');
            set(l,'Fontsize',20);
            hold on;            
        end
        % Plot settings 
        set(gca,'Fontsize',label_fontsize);    
        title(RefData_str_tit{idx_Qs(idx_mtp(i))},'Fontsize',label_fontsize);  
        % Y-axis
        ylim([ylim_Qdots(idx_mtp(i),1) ylim_Qdots(idx_mtp(i),2)]);
        L = get(gca,'YLim');
        set(gca,'YTick',linspace(L(1),L(2),NumTicks_Qs));       
        if i == 1 
            ylabel('(Nm)','Fontsize',label_fontsize);
        end      
        % X-axis
        xlim([Qs_toTrack_deg(1,1),Qs_toTrack_deg(end,1)])
        set(gca,'XTick',[]);
        box off;
    end  
    
end



%% Parameters
for k = 1:length(idx_ww)
    p = 1;
    ww = idx_ww(k);    
    figure() 
    cs = length(ParamsCM_opt(ww).ww(p).p)/3;
    subplot(1,2,1)
    scatter(1:2*cs,ParamsCM_opt(ww).ww(p).p(1:2*cs),'filled','MarkerFaceColor',color_all(2,:),'linewidth',2); hold on
    scatter(1:2*cs,ParamsCM_gen(ww).ww(p).p(1:2*cs),'k','linewidth',2);        
    scatter(1:2*cs,bounds(ww).ww(p).p.params.upper(1:2*cs),'b','linewidth',2);
    scatter(1:2*cs,bounds(ww).ww(p).p.params.lower(1:2*cs),'r','linewidth',2);
    l = legend('Optimal','Generic');
    set(l,'Fontsize',16);
    subplot(1,2,2)
    scatter(1:cs,ParamsCM_opt(ww).ww(p).p(2*cs+1:3*cs),'filled','MarkerFaceColor',color_all(2,:),'linewidth',2); hold on
    scatter(1:cs,ParamsCM_gen(ww).ww(p).p(2*cs+1:3*cs),'k','linewidth',2);    
    scatter(1:cs,bounds(ww).ww(p).p.params.upper(2*cs+1:3*cs),'b','linewidth',2);
    scatter(1:cs,bounds(ww).ww(p).p.params.lower(2*cs+1:3*cs),'r','linewidth',2);
    l = legend('Optimal','Generic');
    set(l,'Fontsize',16);
    if length(ParamsCM_opt(ww).ww(p).p) == 12        
        if writeModel
            pathVariousFunctions = [pathRepo,'\VariousFunctions'];
            addpath(genpath(pathVariousFunctions));    
            pathData = [pathRepo,'\OpenSimModel\',subject];    
            if mtp_pin
                ModelSetup = xml_read([pathData,'\',subject,'_mtpPin_ContactsAsForces.osim']);
            else
                ModelSetup = xml_read([pathData,'\',subject,'_mtp_ContactsAsForces.osim']);
            end
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).location(1) = ParamsCM_opt(ww).ww(p).p(1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).location(3) = ParamsCM_opt(ww).ww(p).p(2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).location(1) = ParamsCM_opt(ww).ww(p).p(3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).location(3) = ParamsCM_opt(ww).ww(p).p(4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(1) = ParamsCM_opt(ww).ww(p).p(5);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(3) = ParamsCM_opt(ww).ww(p).p(6);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).location(1) = ParamsCM_opt(ww).ww(p).p(7);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).location(3) = ParamsCM_opt(ww).ww(p).p(8); 
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(1) = ParamsCM_opt(ww).ww(p).p(1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(3) = -ParamsCM_opt(ww).ww(p).p(2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).location(1) = ParamsCM_opt(ww).ww(p).p(3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).location(3) = -ParamsCM_opt(ww).ww(p).p(4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).location(1) = ParamsCM_opt(ww).ww(p).p(5);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).location(3) = -ParamsCM_opt(ww).ww(p).p(6);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).location(1) = ParamsCM_opt(ww).ww(p).p(7);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).location(3) = -ParamsCM_opt(ww).ww(p).p(8);       
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).radius = ParamsCM_opt(ww).ww(p).p(2*cs+1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).radius = ParamsCM_opt(ww).ww(p).p(2*cs+2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).radius = ParamsCM_opt(ww).ww(p).p(2*cs+3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).radius = ParamsCM_opt(ww).ww(p).p(2*cs+4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).radius = ParamsCM_opt(ww).ww(p).p(2*cs+1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).radius = ParamsCM_opt(ww).ww(p).p(2*cs+2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).radius = ParamsCM_opt(ww).ww(p).p(2*cs+3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).radius = ParamsCM_opt(ww).ww(p).p(2*cs+4);             
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5) = [];
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(6-1) = []; % -1 cause we already removed one
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11-2) = []; % -2 cause we already removed two
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(12-3) = []; % -3 cause we already removed three
            if settings(ww,11) == 2 || settings(ww,11) == 3
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).body_name = 'toes_r';
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(2) = -0.0214476;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).body_name = 'toes_l';
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(2) = -0.0214476;
            end
            ModelSetup.Model.ForceSet.objects.HuntCrossleyForce(5) = [];
            ModelSetup.Model.ForceSet.objects.HuntCrossleyForce(6-1) = []; % -1 cause we already removed one
            ModelSetup.Model.ForceSet.objects.HuntCrossleyForce(11-2) = []; % -2 cause we already removed two
            ModelSetup.Model.ForceSet.objects.HuntCrossleyForce(12-3) = []; % -3 cause we already removed three
            
            PathNewModel = [pathresults,'\',ocp_path,'\',subject,'_mtp_c',num2str(ww),'.osim'];
            xml_writeOSIM(PathNewModel,ModelSetup,'OpenSimDocument');
        end
    elseif length(ParamsCM_opt(ww).ww(p).p) == 15        
        if writeModel
            pathVariousFunctions = [pathRepo,'\VariousFunctions'];
            addpath(genpath(pathVariousFunctions));    
            pathData = [pathRepo,'\OpenSimModel\',subject];    
            if mtp_pin
                ModelSetup = xml_read([pathData,'\',subject,'_mtpPin_ContactsAsForces.osim']);
            else
                ModelSetup = xml_read([pathData,'\',subject,'_mtp_ContactsAsForces.osim']);
            end
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).location(1) = ParamsCM_opt(ww).ww(p).p(1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).location(3) = ParamsCM_opt(ww).ww(p).p(2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).location(1) = ParamsCM_opt(ww).ww(p).p(3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).location(3) = ParamsCM_opt(ww).ww(p).p(4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(1) = ParamsCM_opt(ww).ww(p).p(5);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(3) = ParamsCM_opt(ww).ww(p).p(6);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).location(1) = ParamsCM_opt(ww).ww(p).p(7);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).location(3) = ParamsCM_opt(ww).ww(p).p(8);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).location(1) = ParamsCM_opt(ww).ww(p).p(9);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).location(3) = ParamsCM_opt(ww).ww(p).p(10);  
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(1) = ParamsCM_opt(ww).ww(p).p(1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(3) = -ParamsCM_opt(ww).ww(p).p(2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).location(1) = ParamsCM_opt(ww).ww(p).p(3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).location(3) = -ParamsCM_opt(ww).ww(p).p(4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).location(1) = ParamsCM_opt(ww).ww(p).p(5);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).location(3) = -ParamsCM_opt(ww).ww(p).p(6);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).location(1) = ParamsCM_opt(ww).ww(p).p(7);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).location(3) = -ParamsCM_opt(ww).ww(p).p(8);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).location(1) = ParamsCM_opt(ww).ww(p).p(9);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).location(3) = -ParamsCM_opt(ww).ww(p).p(10);        
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).radius = ParamsCM_opt(ww).ww(p).p(2*cs+1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).radius = ParamsCM_opt(ww).ww(p).p(2*cs+2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).radius = ParamsCM_opt(ww).ww(p).p(2*cs+3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).radius = ParamsCM_opt(ww).ww(p).p(2*cs+4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).radius = ParamsCM_opt(ww).ww(p).p(2*cs+5);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).radius = ParamsCM_opt(ww).ww(p).p(2*cs+1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).radius = ParamsCM_opt(ww).ww(p).p(2*cs+2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).radius = ParamsCM_opt(ww).ww(p).p(2*cs+3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).radius = ParamsCM_opt(ww).ww(p).p(2*cs+4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).radius = ParamsCM_opt(ww).ww(p).p(2*cs+5);               
            if settings(ww,11) == 2 || settings(ww,11) == 3
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).body_name = 'toes_r';
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).location(2) = -0.0214476;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).body_name = 'toes_l';
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).location(2) = -0.0214476;
            end                        
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(6) = [];
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(12-1) = []; % -1 cause we already removed one
            ModelSetup.Model.ForceSet.objects.HuntCrossleyForce(6) = [];
            ModelSetup.Model.ForceSet.objects.HuntCrossleyForce(12-1) = []; % -1 cause we already removed one
            PathNewModel = [pathresults,'\',ocp_path,'\',subject,'_mtp_c',num2str(ww),'.osim'];
            xml_writeOSIM(PathNewModel,ModelSetup,'OpenSimDocument');
        end
    elseif length(ParamsCM_opt(ww).ww(p).p) == 18
        if writeModel
            pathVariousFunctions = [pathRepo,'\VariousFunctions'];
            addpath(genpath(pathVariousFunctions));    
            pathData = [pathRepo,'\OpenSimModel\',subject];    
            if mtp_pin
                ModelSetup = xml_read([pathData,'\',subject,'_mtpPin_ContactsAsForces.osim']);
            else
                ModelSetup = xml_read([pathData,'\',subject,'_mtp_ContactsAsForces.osim']);
            end
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).location(1) = ParamsCM_opt(ww).ww(p).p(1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).location(3) = ParamsCM_opt(ww).ww(p).p(2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).location(1) = ParamsCM_opt(ww).ww(p).p(3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).location(3) = ParamsCM_opt(ww).ww(p).p(4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(1) = ParamsCM_opt(ww).ww(p).p(5);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(3) = ParamsCM_opt(ww).ww(p).p(6);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).location(1) = ParamsCM_opt(ww).ww(p).p(7);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).location(3) = ParamsCM_opt(ww).ww(p).p(8);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).location(1) = ParamsCM_opt(ww).ww(p).p(9);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).location(3) = ParamsCM_opt(ww).ww(p).p(10);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(6).location(1) = ParamsCM_opt(ww).ww(p).p(11);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(6).location(3) = ParamsCM_opt(ww).ww(p).p(12);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(1) = ParamsCM_opt(ww).ww(p).p(1);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(3) = -ParamsCM_opt(ww).ww(p).p(2);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).location(1) = ParamsCM_opt(ww).ww(p).p(3);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).location(3) = -ParamsCM_opt(ww).ww(p).p(4);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).location(1) = ParamsCM_opt(ww).ww(p).p(5);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).location(3) = -ParamsCM_opt(ww).ww(p).p(6);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).location(1) = ParamsCM_opt(ww).ww(p).p(7);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).location(3) = -ParamsCM_opt(ww).ww(p).p(8);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).location(1) = ParamsCM_opt(ww).ww(p).p(9);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).location(3) = -ParamsCM_opt(ww).ww(p).p(10);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(12).location(1) = ParamsCM_opt(ww).ww(p).p(11);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(12).location(3) = -ParamsCM_opt(ww).ww(p).p(12);            
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).radius = ParamsCM_opt(ww).ww(p).p(13);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).radius = ParamsCM_opt(ww).ww(p).p(14);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).radius = ParamsCM_opt(ww).ww(p).p(15);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).radius = ParamsCM_opt(ww).ww(p).p(16);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).radius = ParamsCM_opt(ww).ww(p).p(17);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(6).radius = ParamsCM_opt(ww).ww(p).p(18);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).radius = ParamsCM_opt(ww).ww(p).p(13);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).radius = ParamsCM_opt(ww).ww(p).p(14);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).radius = ParamsCM_opt(ww).ww(p).p(15);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).radius = ParamsCM_opt(ww).ww(p).p(16);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).radius = ParamsCM_opt(ww).ww(p).p(17);
            ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(12).radius = ParamsCM_opt(ww).ww(p).p(18);
            if settings(ww,11) == 4 || settings(ww,11) == 5 || settings(ww,11) == 6
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(1).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(2).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(3).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(6).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(7).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(8).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(9).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(12).location(2) = -0.01;
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(4).body_name = 'calcn_r';
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(10).body_name = 'calcn_l';
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(5).body_name = 'toes_r';
                ModelSetup.Model.ContactGeometrySet.objects.ContactSphere(11).body_name = 'toes_l';
            end
            PathNewModel = [pathresults,'\',ocp_path,'\',subject,'_mtp_c',num2str(ww),'.osim'];
            xml_writeOSIM(PathNewModel,ModelSetup,'OpenSimDocument');
        end
    end
end
