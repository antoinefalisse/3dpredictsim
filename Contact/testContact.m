clear all
clc

import casadi.*

%% Indices external function
pathmain = pwd;
F1 = external('F','subject1_2D_track1.dll'); 

vec1 = -ones(24,1);
outk1 = full(F1(vec1));

% External function: F1
calcn.l.omega = 1:3;
calcn.l.v_lin = 4:6;
calcn.r.omega = 7:9;
calcn.r.v_lin = 10:12;
calcn.l.pos = 13:15;
calcn.r.pos = 16:18;
calcn.TR.R.l = 19:27;
calcn.TR.T.l = 28:30;
calcn.TR.R.r = 31:39;
calcn.TR.T.r = 40:42;
toes.l.omega = 43:45;
toes.l.v_lin = 46:48;
toes.r.omega = 49:51;
toes.r.v_lin = 52:54;
toes.l.pos = 55:57;
toes.r.pos = 58:60;
toes.TR.R.l = 61:69;
toes.TR.T.l = 70:72;
toes.TR.R.r = 73:81;
toes.TR.T.r = 82:84;

stiffness = 1000000;
radius = 0.035;
dissipation = 2;
normal = [0,1,0];
transitionVelocity = 0.2;
staticFriction = 0.8;
dynamicFriction = 0.8;
viscousFriction = 0.5; 
     
spherePos_inB = [0.01, 0, 0]'; 
posFrame_inG = outk1(calcn.l.pos);
linVelFrame_inG = outk1(calcn.l.v_lin);
angVelFrame_InG = outk1(calcn.l.omega);
rotBtoG_inG = outk1(calcn.TR.R.l);
trBtoG_inG = outk1(calcn.TR.T.l);

force = HCContactModel(stiffness,radius,dissipation,normal,...
    transitionVelocity,staticFriction,dynamicFriction,viscousFriction,...
    spherePos_inB,posFrame_inG,linVelFrame_inG,angVelFrame_InG,...
    rotBtoG_inG,trBtoG_inG); 