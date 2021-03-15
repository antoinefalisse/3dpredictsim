% This script returns the timing of the initial contacts. leg indicates
% which leg should be in contact with the ground first
% We want to simulate a half gait cycle so for instance from inital contact
% of the right leg until initial contact of the left leg.

function [time_IC,idx_IC,leg1] = getIC_1FP(GRF,threshold,timeIC2)

logic_L = GRF.val.l(:,2) >= threshold;
logic_R = GRF.val.r(:,2) >= threshold;

GRF_L_IC = find(diff(logic_L)==1) +1;
GRF_R_IC = find(diff(logic_R)==1) +1;

idx_IC2 = find(round(GRF.val.all(:,1),4)==round(timeIC2,2));

if GRF_L_IC < GRF_R_IC
   time_IC(1) =  GRF.val.all(GRF_L_IC,1);
   idx_IC(1) = GRF_L_IC;   
   leg1 = 'l';
else
   time_IC(1) =  GRF.val.all(GRF_R_IC,1);   
   idx_IC(1) = GRF_R_IC;
   leg1 = 'r';
end
time_IC(2) =  round(timeIC2,2);
idx_IC(2) = idx_IC2;
   
