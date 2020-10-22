% This script reproduces Fig 2
% Author: Antoine Falisse
% Date: 1/7/2019

clear all
close all
clc

%% Settings
% Selected trials
% Trials at all speeds from different initial guesses
ww  = [1:75,77:82,84:103];
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
all_speeds          = zeros(1,length(ww));
all_costs           = zeros(1,length(ww));
% Loop over cases
for k = 1:length(ww)
    predSim_data_all;
    all_speeds(k) = settings(ww(k),1);
    all_costs(k) = Cost(ww(k)).m;
end

%% Select result with lowest optimal cost for each speed
[all_speeds_s,all_speedsi_s] = sort(all_speeds);
all_costs_s = all_costs(all_speedsi_s);   
all_speeds_s_u = unique(all_speeds_s);
[Nspeed,~] = ...
    histcounts(all_speeds_s,[all_speeds_s_u all_speeds_s_u(end)+0.1]);
count = 0;
cost_min = zeros(1,length(Nspeed));
cost_mini = zeros(1,length(Nspeed));
for i = 1:length(Nspeed)
    cost_min_temp = zeros(1,Nspeed(i));
    for j = 1:Nspeed(i)
        cost_min_temp(j) = all_costs_s(count+j);        
    end
    cost_min(i) = min(cost_min_temp);
    cost_mini(i) = find(all_costs==cost_min(i));
    count = count + Nspeed(i);    
end
trials_lowestCost = ww(cost_mini);

%% Detect walk-to-run transition speed
ds = zeros(1,length(Nspeed));
ds_bol = zeros(1,length(Nspeed));
for k = 1:length(Nspeed)
        % Is there a double support phase?
        temp_r = GRFs_opt(ww(cost_mini(k))).m(:,2).*(body_weight/100);
        temp_l = GRFs_opt(ww(cost_mini(k))).m(:,5).*(body_weight/100);
        threshold = 20;
        ds(k) = sum(temp_l(temp_r>threshold));
        ds_bol(k) = ds(k) < 1100;  
end 
if sum(ds_bol) ~= 0         
    speed_tr = all_speeds_s_u(find(ds_bol,1,'first'));
    disp(['Transition speed: ',num2str(speed_tr)])
    if sum(ds_bol(find(ds_bol,1,'first'):end)) == ...
            length(ds_bol(find(ds_bol,1,'first'):end))
        disp('All running from transition speed')
    else
        disp('NOT all running from transition speed')
    end
end

%% Common settings for plots
label_fontsize  = 16;
line_linewidth  = 2;
scatter_size = 40;
% Colors
color1 = [244,194,13]/255;     % Yellow
color2 = [0,0,0];              % Black
color3 = [219,50,54]/255;      % Red
color4 = [72,133,237]/255;     % Blue
colorexp = [60,186,84]/255;    % Green

noc = length(0.73:0.1:1.23) + 1;
color_slow = [linspace(color1(1),color2(1),noc)',...
    linspace(color1(2),color2(2),noc)',linspace(color1(3),color2(3),noc)'];
color_slow = color_slow(1:end-1,:);

noc = length(1.33:0.1:speed_tr-0.1) + 1 ;
color_mid = [linspace(color2(1),color3(1),noc)',...
    linspace(color2(2),color3(2),noc)',linspace(color2(3),color3(3),noc)'];
color_mid = color_mid(1:end-1,:);

noc = length(speed_tr:0.1:2.73);
color_fast = [linspace(color3(1),color4(1),noc)',...
    linspace(color3(2),color4(2),noc)',linspace(color3(3),color4(3),noc)'];
% color_fast = color_fast(1:end-1,:);

%% Metabolic cost of transport (COT) versus speed
ylim_COT = [3 5]; 
xlim_COT = [0.5 3];
NumTicks_YCOT = 3;
NumTicks_XCOT = 6;
figure()
subplot(2,2,1)
count1 = 1;
count2 = 1;
count3 = 1;
y = zeros(1,length(Nspeed));
for k = 1:length(Nspeed)
    if settings(ww(cost_mini(k)),1) < 1.33  
        scatter(settings(ww(cost_mini(k)),1),...
            COT_opt(ww(cost_mini(k))).m,scatter_size,...
            color_slow(count1,:),'filled');
        hold on; 
        y(k) = COT_opt(ww(cost_mini(k))).m;
        count1 = count1 + 1;    
    elseif settings(ww(cost_mini(k)),1) < speed_tr 
        scatter(settings(ww(cost_mini(k)),1),...
        COT_opt(ww(cost_mini(k))).m,scatter_size,...
        color_mid(count2,:),'filled','d');
        hold on; 
        y(k) = COT_opt(ww(cost_mini(k))).m;
        count2 = count2 + 1;    
    else
        scatter(settings(ww(cost_mini(k)),1),...
            COT_opt(ww(cost_mini(k))).m,scatter_size,...
            color_fast(count3,:),'filled','s');
        hold on; 
        y(k) = COT_opt(ww(cost_mini(k))).m;   
        count3 = count3 + 1;
    end
end
% Quadratic regression for walking speeds
idx_tr = find(all_speeds_s_u==speed_tr);
x = all_speeds_s_u(1):0.1:all_speeds_s_u(idx_tr);
p = polyfit(x,y(1:idx_tr),2);
step_q = (all_speeds_s_u(idx_tr)-all_speeds_s_u(1))/...
    ((all_speeds_s_u(idx_tr)-all_speeds_s_u(1))*100);
x1_q = all_speeds_s_u(1):step_q:all_speeds_s_u(idx_tr);
f1_q = polyval(p,x1_q);
plot(x1_q,f1_q,'color','k','linewidth',line_linewidth);
% Linear regression for running speeds
x = all_speeds_s_u(idx_tr):0.1:all_speeds_s_u(end);
X = [ones(length(x),1) x'];
b = X\y(idx_tr:end)';
yy = X*b;
plot(x,yy,'color','k','linewidth',line_linewidth);
% Vertical line: preferred walking speed
plot([1.33,1.33],ylim_COT,'color',[0,0,0],'linestyle','--',...
    'linewidth',line_linewidth);
% Vertical line: walk-to-run transition speed
plot([all_speeds_s_u(idx_tr),all_speeds_s_u(idx_tr)],ylim_COT,...
    'color',color3,'linestyle',':','linewidth',line_linewidth);
% Plot settings   
set(gca,'Fontsize',label_fontsize);
% Y-axis
ylim(ylim_COT);
L = get(gca,'YLim');
set(gca,'YTick',linspace(L(1),L(2),NumTicks_YCOT))
ylabel('COT (J kg-1 m-1)','Fontsize',label_fontsize);
% X-axis
xlim(xlim_COT);
L = get(gca,'XLim');
set(gca,'XTick',linspace(L(1),L(2),NumTicks_XCOT))
xlabel('Speed (m s-1)','Fontsize',label_fontsize);
box off

%% Frequency versus speed
ylim_fr = [0.6 1.6];
xlim_fr = [0.5,3];
NumTicks_Yfr = 3;
NumTicks_Xfr = 6;
% Plot
subplot(2,2,2)
count1 = 1;
count2 = 1;
count3 = 1;
count4 = 1;
freq_walk = zeros(1,length(Nspeed));
for k = 1:length(Nspeed)    
    if settings(ww(cost_mini(k)),1) < 1.33        
        freq_walk(count1) = ...
        settings(ww(cost_mini(k)),1)/StrideLength_opt(ww(cost_mini(k))).m;
        scatter((settings(ww(cost_mini(k)),1)),freq_walk(count1),...
            scatter_size,color_slow(count2,:),'filled');
        count2 = count2 + 1;
    elseif settings(ww(cost_mini(k)),1) < speed_tr       
        freq_walk(count1) = ...
        settings(ww(cost_mini(k)),1)/StrideLength_opt(ww(cost_mini(k))).m;
        scatter((settings(ww(cost_mini(k)),1)),freq_walk(count1),...
            scatter_size,color_mid(count3,:),'filled','d');        
        count3 = count3 + 1;
    else
        freq_walk(count1) = ...
        settings(ww(cost_mini(k)),1)/StrideLength_opt(ww(cost_mini(k))).m;
        scatter((settings(ww(cost_mini(k)),1)),freq_walk(count1),...
            scatter_size,color_fast(count4,:),'filled','s');
        count4 = count4 + 1;
    end
    count1 = count1 + 1;    
    hold on;      
end
% Linear regressions for walking
% Simulation results
freq_walk = freq_walk(1:idx_tr);
x = (all_speeds_s_u(1):0.1:all_speeds_s_u(idx_tr));
X = [ones(length(x),1) x'];
b = X\freq_walk';
yy = X*b;
plot(x,yy,'k','linewidth',line_linewidth);
% Experimental data from Hansen et al. (2017)
b_exp = [34.2,4.5]';
x_kmh = (all_speeds_s_u(1):0.1:all_speeds_s_u(idx_tr))*3.6;
X_kmh = [ones(length(x_kmh),1) x_kmh'];
yyexp_min = X_kmh*b_exp;
% Convert in strides s-1 and m s-1
yyexp_s = yyexp_min./60;
b_kmh  = X\yyexp_s;
plot(x,yyexp_s,'color',colorexp,'linestyle','--','linewidth',line_linewidth);
% Vertical line: walk-to-run transition speed
plot([all_speeds_s_u(idx_tr),all_speeds_s_u(idx_tr)],ylim_fr,...
    'color',color3,'linestyle',':','linewidth',line_linewidth);
% Plot settings 
set(gca,'Fontsize',label_fontsize);
% Y-axis
ylim(ylim_fr);
L = get(gca,'YLim');
set(gca,'YTick',linspace(L(1),L(2),NumTicks_Yfr))
ylabel('Frequency (strides s-1)','Fontsize',label_fontsize);
% X-axis
xlim(xlim_fr);
L = get(gca,'XLim');
set(gca,'XTick',linspace(L(1),L(2),NumTicks_Xfr))
xlabel('Speed (m s-1)','Fontsize',label_fontsize);
box off

%% Ground reaction forces
ylim_GRF = [0 200];
NumTicks_YGRF = 3;
NumTicks_XGRF = 3;
% Plot 1
count1 = 1;
subplot(2,3,4)
for k = 1:length(Nspeed)
    x = 1:(100-1)/(size(GRFs_opt(ww(cost_mini(k))).m,1)-1):100;
    if settings(ww(cost_mini(k)),1) < 1.33
        plot(x,GRFs_opt(ww(cost_mini(k))).m(:,2),'color',...
            color_slow(count1,:),'linewidth',line_linewidth);
        count1 = count1 + 1;
    end
    hold on;        
end  
% Plot settings
set(gca,'Fontsize',label_fontsize);
% Y-axis
ylim(ylim_GRF);
L = get(gca,'YLim');
ylabel('Vertical force (%BW)','Fontsize',label_fontsize);
set(gca,'YTick',linspace(L(1),L(2),NumTicks_YGRF))
% X-axis
L = get(gca,'XLim');
set(gca,'XTick',linspace(L(1),L(2),NumTicks_XGRF))
xlabel('Gait cycle (%)','Fontsize',label_fontsize);
% colormap(color_slow)
% colorbar
box off
% Plot 2
count2 = 1;
subplot(2,3,5)
for k = 1:length(Nspeed)
    x = 1:(100-1)/(size(GRFs_opt(ww(cost_mini(k))).m,1)-1):100;
    if settings(ww(cost_mini(k)),1) >= 1.33 && ...
            settings(ww(cost_mini(k)),1) < speed_tr
        plot(x,GRFs_opt(ww(cost_mini(k))).m(:,2),'color',...
            color_mid(count2,:),'linewidth',line_linewidth);
        count2 = count2 + 1;
    end
    hold on;        
end 
% Plot settings
set(gca,'Fontsize',label_fontsize);
% Y-axis
ylim(ylim_GRF);
L = get(gca,'YLim');
set(gca,'YTick',linspace(L(1),L(2),NumTicks_YGRF))
% X-axis
L = get(gca,'XLim');
set(gca,'XTick',linspace(L(1),L(2),NumTicks_XGRF))
xlabel('Gait cycle (%)','Fontsize',label_fontsize);
% colormap(color_mid)
% colorbar
box off
% Plot 3
count3 = 1;
subplot(2,3,6)
for k = 1:length(Nspeed)
    x = 1:(100-1)/(size(GRFs_opt(ww(cost_mini(k))).m,1)-1):100;
    if settings(ww(cost_mini(k)),1) >= speed_tr
        plot(x,GRFs_opt(ww(cost_mini(k))).m(:,2),'color',...
            color_fast(count3,:),'linewidth',line_linewidth);
        count3 = count3 + 1;
    end        
    hold on;  
end 
% Plot settings
set(gca,'Fontsize',label_fontsize);
% Y-axis
ylim(ylim_GRF);
L = get(gca,'YLim');
set(gca,'YTick',linspace(L(1),L(2),NumTicks_YGRF))
% X-axis
L = get(gca,'XLim');
set(gca,'XTick',linspace(L(1),L(2),NumTicks_XGRF))
xlabel('Gait cycle (%)','Fontsize',label_fontsize);
% colormap(color_fast)
% colorbar
box off
