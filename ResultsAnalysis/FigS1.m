% This script reproduces Fig S1
% Author: Antoine Falisse
% Date: 1/7/2019

clear all
close all
clc

%% Settings
% Selected trials
% 14,77,16,17: increasing speeds (2.03 to 2.33 ms-1)
% with generic maximal contraction velocities (10*lMopt)
% 173,174,175,176: increasing speeds (2.03 to 2.33 ms-1)
% with doubled maximal contraction velocities (20*lMopt) 
ww  = [14,77,16,17,173,174,175,176]; 
% Fixed settings
subject = 'subject1';
body_mass = 62;
body_weight = 62*9.81;
g = 9.81;
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
COM_p_y             = struct('m',[]);
COM_v_x             = struct('m',[]);
% Loop over cases
for k = 1:length(ww)
    predSim_data_all;
    % Extract also COM vertical position and forward velocity (post-processing)
    COM_p_y(ww(k)).m = Results_all.(['Speed_',num2str(v_tgt_id*100)]). ...  
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
        (['CoContraction_',num2str(coCont)]).COM_p_y;
    COM_v_x(ww(k)).m = Results_all.(['Speed_',num2str(v_tgt_id*100)]). ...  
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
        (['CoContraction_',num2str(coCont)]).COM_v_x;
end

%% Common settings for plots
label_fontsize  = 16;
line_linewidth  = 3;
% Colors
color_all(1,:)  = [0,0,0];              % Black
color_all(2,:)  = [219,50,54]/255;      % Red

%% Ground reaction forces
GRF_str = {'Fore-aft','Vertical','Lateral'};
ylim_GRF = ([-50,50;0 200;-25 25]);
pos_GRF = [5:8,5:8];
NumTicks_GRF = 2;
N = length(pos_GRF)/2;
title_str = {'2.03 ms-1','2.13 ms-1','2.23 ms-1','2.33 ms-1'};
figure()
for k = 1:length(ww)
    subplot(3,4,pos_GRF(k))     
    x = 1:(100-1)/(size(GRFs_opt(ww(k)).m,1)-1):100;
    if k <= N            
        plot(x,GRFs_opt(ww(k)).m(:,2),'color',...
            color_all(1,:),'linewidth',line_linewidth);
    else
        plot(x,GRFs_opt(ww(k)).m(:,2),'color',...
            color_all(2,:),'linestyle',':','linewidth',line_linewidth);  
    end
    hold on;     
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);    
    % Y-axis
    ylim([ylim_GRF(2,1) ylim_GRF(2,2)]);
    L = get(gca,'YLim');
    set(gca,'YTick',linspace(L(1),L(2),NumTicks_GRF));       
    if k == 1 
        ylabel('Force (%BW)','Fontsize',label_fontsize);
    end      
    % X-axis    
    set(gca,'XTick',[]);
    box off;       
end    

%% Gravitational potential and forward kinetic energies
% Generic maximal muscle contraction velocities
pos_E = 1:4;
for k = 1:length(ww)/2    
    % Gravitational potential energy
    % Compute gravitational potential energy
    E_p = COM_p_y(ww(k)).m*body_mass*g;
    E_p_shift = E_p - min(E_p);
    E_p_shift_norm = E_p_shift/max(E_p_shift);    
    % Forward kinetic energy
    % Compute forward kinetic energy
    E_kf = 0.5*body_mass*COM_v_x(ww(k)).m.^2;
    E_kf_shift = E_kf - min(E_kf); 
    E_kf_shift_norm = E_kf_shift/max(E_kf_shift);
    % Plots
    subplot(3,4,pos_E(k))
    plot(E_p_shift_norm,'k','linewidth',line_linewidth);
    hold on
    plot(E_kf_shift_norm,'k--','linewidth',line_linewidth);
    plot([1 100],[0 0],'k','linewidth',line_linewidth);
    plot([1 100],[1 1],'k','linewidth',line_linewidth);
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);  
    title(title_str{k},'Fontsize',label_fontsize);
    % Y-axis
    ylim([0 1]);
    L = get(gca,'YLim');
    set(gca,'YTick',linspace(L(1),L(2),NumTicks_GRF));
    set(gca,'YTickLabel',{'min','max'},'Fontsize',label_fontsize');
    if k == 1 
        ylabel('Mechanical energy','Fontsize',label_fontsize);
    end      
    % X-axis    
    set(gca,'XTick',[]);
    box off;       
end

% Increased maximal muscle contraction velocities
pos_E = 9:12;
for k = 1:length(ww)/2    
    % Gravitational potential energy
    % Compute gravitational potential energy
    E_p = COM_p_y(ww(k+4)).m*body_mass*g;
    E_p_shift = E_p - min(E_p);
    E_p_shift_norm = E_p_shift/max(E_p_shift);    
    % Forward kinetic energy
    % Compute forward kinetic energy
    E_kf = 0.5*body_mass*COM_v_x(ww(k+4)).m.^2;
    E_kf_shift = E_kf - min(E_kf); 
    E_kf_shift_norm = E_kf_shift/max(E_kf_shift);
    % Plots
    subplot(3,4,pos_E(k))
    plot(E_p_shift_norm,'color',color_all(2,:),'linewidth',line_linewidth);
    hold on
    plot(E_kf_shift_norm,'color',color_all(2,:),'linestyle','--','linewidth',line_linewidth);
    plot([1 100],[0 0],'k','linewidth',line_linewidth);
    plot([1 100],[1 1],'k','linewidth',line_linewidth);
    % Plot settings 
    set(gca,'Fontsize',label_fontsize);  
    % Y-axis
    ylim([0 1]);
    L = get(gca,'YLim');
    set(gca,'YTick',linspace(L(1),L(2),NumTicks_GRF));
    set(gca,'YTickLabel',{'min','max'},'Fontsize',label_fontsize');
    if k == 1 
        ylabel('Mechanical energy','Fontsize',label_fontsize);
    end      
    % X-axis    
    L = get(gca,'XLim');
    set(gca,'XTick',linspace(L(1),L(2),NumTicks_GRF))
    xlabel('Gait cycle (%)','Fontsize',label_fontsize);
    box off;       
end
