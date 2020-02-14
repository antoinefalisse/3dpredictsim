% This script contains several CasADi-based functions that are
% used when solving the OCPs
%
% Author: Antoine Falisse
% Date: 12/19/2018
%
import casadi.*

%% Polynomial approximation
pathpolynomial = [pathRepo,'/Polynomials'];
addpath(genpath(pathpolynomial));
muscle_spanning_info_m = muscle_spanning_joint_INFO(musi_pol,:);
MuscleInfo_m.muscle    = MuscleInfo.muscle(musi_pol);                  
qin     = SX.sym('qin',1,nq.leg);
qdotin  = SX.sym('qdotin',1,nq.leg);
lMT     = SX(NMuscle_pol,1);
vMT     = SX(NMuscle_pol,1);
dM      = SX(NMuscle_pol,nq.leg);
for i=1:NMuscle_pol      
    index_dof_crossing  = find(muscle_spanning_info_m(i,:)==1);
    order               = MuscleInfo_m.muscle(i).order;
    [mat,diff_mat_q]    = n_art_mat_3_cas_SX(qin(1,index_dof_crossing),...
        order);
    lMT(i,1)            = mat*MuscleInfo_m.muscle(i).coeff;
    vMT(i,1)            = 0;
    dM(i,1:nq.leg)      = 0;
    nr_dof_crossing     = length(index_dof_crossing); 
    for dof_nr = 1:nr_dof_crossing
        dM(i,index_dof_crossing(dof_nr)) = ...
            (-(diff_mat_q(:,dof_nr)))'*MuscleInfo_m.muscle(i).coeff;
        vMT(i,1) = vMT(i,1) + (-dM(i,index_dof_crossing(dof_nr))*...
            qdotin(1,index_dof_crossing(dof_nr)));
    end 
end
f_lMT_vMT_dM = Function('f_lMT_vMT_dM',{qin,qdotin},{lMT,vMT,dM});

%% Normalized sum of squared values
% Function for 6 elements 
etemp6 = SX.sym('etemp6',6);
Jtemp6 = 0;
for i=1:length(etemp6)
    Jtemp6 = Jtemp6 + etemp6(i).^2;
end
Jtemp6 = Jtemp6/6;
f_J6 = Function('f_J6',{etemp6},{Jtemp6});
% Function for 92 elements 
etemp92 = SX.sym('etemp92',92);
Jtemp92 = 0;
for i=1:length(etemp92)
    Jtemp92 = Jtemp92 + etemp92(i).^2;
end
Jtemp92 = Jtemp92/92;
f_J92 = Function('f_J92',{etemp92},{Jtemp92});
% Function for 30 elements
etemp30 = SX.sym('etemp30',30);
Jtemp30 = 0;
for i=1:length(etemp30)
    Jtemp30 = Jtemp30 + etemp30(i).^2;
end
Jtemp30 = Jtemp30/30;
f_J30 = Function('f_J30',{etemp30},{Jtemp30});
% Function for 25 elements
etemp25 = SX.sym('etemp25',25);
Jtemp25 = 0;
for i=1:length(etemp25)
    Jtemp25 = Jtemp25 + etemp25(i).^2;
end
Jtemp25 = Jtemp25/25;
f_J25 = Function('f_J25',{etemp25},{Jtemp25});
% Function for 23 elements
etemp23 = SX.sym('etemp23',23);
Jtemp23 = 0;
for i=1:length(etemp23)
    Jtemp23 = Jtemp23 + etemp23(i).^2;
end
Jtemp23 = Jtemp23/23;
f_J23 = Function('f_J23',{etemp23},{Jtemp23});
% Function for 31 elements
etemp31 = SX.sym('etemp31',31);
Jtemp31 = 0;
for i=1:length(etemp31)
    Jtemp31 = Jtemp31 + etemp31(i).^2;
end
Jtemp31 = Jtemp31/31;
f_J31 = Function('f_J31',{etemp31},{Jtemp31});
% Function for 2 elements
etemp2 = SX.sym('etemp2',2);
Jtemp2 = 0;
for i=1:length(etemp2)
    Jtemp2 = Jtemp2 + etemp2(i).^2;
end
Jtemp2 = Jtemp2/2;
f_J2 = Function('f_J2',{etemp2},{Jtemp2});
% Function for 8 elements
etemp8 = SX.sym('etemp8',8);
Jtemp8 = 0;
for i=1:length(etemp8)
    Jtemp8 = Jtemp8 + etemp8(i).^2;
end
Jtemp8 = Jtemp8/8;
f_J8 = Function('f_J8',{etemp8},{Jtemp8});

%% Sum of products 
% Function for 27 elements 
ma_temp27 = SX.sym('ma_temp27',27);
ft_temp27 = SX.sym('ft_temp27',27);
J_sptemp27 = 0;
for i=1:length(ma_temp27)
    J_sptemp27 = J_sptemp27 + ma_temp27(i,1)*ft_temp27(i,1);    
end
f_T27 = Function('f_T27',{ma_temp27,ft_temp27},{J_sptemp27});
% Function for 13 elements 
ma_temp13 = SX.sym('ma_temp13',13);
ft_temp13 = SX.sym('ft_temp13',13);
J_sptemp13 = 0;
for i=1:length(ma_temp13)
    J_sptemp13 = J_sptemp13 + ma_temp13(i,1)*ft_temp13(i,1);    
end
f_T13 = Function('f_T13',{ma_temp13,ft_temp13},{J_sptemp13});
% Function for 12 elements 
ma_temp12 = SX.sym('ma_temp12',12);
ft_temp12 = SX.sym('ft_temp12',12);
J_sptemp12 = 0;
for i=1:length(ma_temp12)
    J_sptemp12 = J_sptemp12 + ma_temp12(i,1)*ft_temp12(i,1);    
end
f_T12 = Function('f_T12',{ma_temp12,ft_temp12},{J_sptemp12});
% Function for 6 elements 
ma_temp6 = SX.sym('ma_temp6',6);
ft_temp6 = SX.sym('ft_temp6',6);
J_sptemp6 = 0;
for i=1:length(ma_temp6)
    J_sptemp6 = J_sptemp6 + ma_temp6(i,1)*ft_temp6(i,1);    
end
f_T6 = Function('f_T6',{ma_temp6,ft_temp6},{J_sptemp6});

%% Arm activation dynamics
e_a = SX.sym('e_a',nq.arms); % arm excitations
a_a = SX.sym('a_a',nq.arms); % arm activations
dadt = ArmActivationDynamics(e_a,a_a);
f_ArmActivationDynamics = ...
    Function('f_ArmActivationDynamics',{e_a,a_a},{dadt});

%% Mtp activation dynamics
e_a = SX.sym('e_a',nq.mtp); % arm excitations
a_a = SX.sym('a_a',nq.mtp); % arm activations
dadt = ArmActivationDynamics(e_a,a_a);
f_MtpActivationDynamics = ...
    Function('f_MtpActivationDynamics',{e_a,a_a},{dadt});

%% Muscle contraction dynamics
pathmusclemodel = [pathRepo,'/MuscleModel'];
addpath(genpath(pathmusclemodel));
% Function for Hill-equilibrium
FTtilde     = SX.sym('FTtilde',NMuscle); % Normalized tendon forces
a           = SX.sym('a',NMuscle); % Muscle activations
dFTtilde    = SX.sym('dFTtilde',NMuscle); % Time derivative tendon forces
lMT         = SX.sym('lMT',NMuscle); % Muscle-tendon lengths
vMT         = SX.sym('vMT',NMuscle); % Muscle-tendon velocities
tension_SX  = SX.sym('tension',NMuscle); % Tensions
Hilldiff    = SX(NMuscle,1); % Hill-equilibrium
FT          = SX(NMuscle,1); % Tendon forces
Fce         = SX(NMuscle,1); % Contractile element forces
Fiso        = SX(NMuscle,1); % Normalized forces from force-length curve
vMmax       = SX(NMuscle,1); % Maximum contraction velocities
massM       = SX(NMuscle,1); % Muscle mass
% Parameters of force-length-velocity curves
load Fvparam
load Fpparam
load Faparam
for m = 1:NMuscle
    [Hilldiff(m),FT(m),Fce(m),Fiso(m),vMmax(m),massM(m)] = ...
        ForceEquilibrium_FtildeState_all(a(m),FTtilde(m),dFTtilde(m),...
        lMT(m),vMT(m),MTparameters_m(:,m),Fvparam,Fpparam,Faparam,...
        tension_SX(m));
end
f_forceEquilibrium_FtildeState_all = ...
    Function('f_forceEquilibrium_FtildeState_all',{a,FTtilde,dFTtilde,...
    lMT,vMT,tension_SX},{Hilldiff,FT,Fce,Fiso,vMmax,massM});

%% Passive joint torques
K_pass      = SX.sym('K_pass',4);
theta_pass  = SX.sym('theta_pass',2);
qin_pass    = SX.sym('qin_pass',1);
qdotin_pass = SX.sym('qdotin_pass',1);
% theta_pass 1 and 2 are inverted on purpose.
Tau_pass = K_pass(1,1)*exp(K_pass(2,1)*(qin_pass-theta_pass(2,1))) + ...
    K_pass(3,1)*exp(K_pass(4,1)*(qin_pass-theta_pass(1,1))) ...
    - 0.1*qdotin_pass;
f_PassiveMoments = Function('f_PassiveMoments',{K_pass,theta_pass,...
    qin_pass,qdotin_pass},{Tau_pass});

%% Passive torque actuated joint torques
stiff	= SX.sym('stiff',1);
damp	= SX.sym('damp',1);
qin     = SX.sym('qin_pass',1);
qdotin  = SX.sym('qdotin_pass',1);
passTATorques = -stiff * qin - damp * qdotin;
f_passiveTATorques = Function('f_passiveTATorques',{stiff,damp,qin,qdotin}, ...
    {passTATorques});

%% Unscaling
x_barnp      = SX.sym('x_barnp_SX',np,1); 
sc_vnp       = SX.sym('sc_vnp_SX',np,1);
sc_rnp       = SX.sym('sc_rnp',np,1);
xnp          = (x_barnp - sc_rnp)./sc_vnp;
f_nscnp      = Function('f_nscnp',{x_barnp,sc_vnp,sc_rnp},{xnp});

%% Contact forces
stiffnessSX         = SX.sym('stiffnessSX',1);
radiusSX            = SX.sym('radiusSX',1);
dissipationSX       = SX.sym('dissipationSX',1);
normalSX            = SX.sym('normalSX',1,3);
transitionVelocitySX= SX.sym('transitionVelocitySX',1);
staticFrictionSX    = SX.sym('staticFrictionSX',1);
dynamicFrictionSX   = SX.sym('dynamicFrictionSX',1);
viscousFrictionSX   = SX.sym('viscousFrictionSX',1);
spherePosSX         = SX.sym('spherePosSX',3);
orFramePosSX        = SX.sym('orFramePosSX',3);
v_linSX             = SX.sym('v_linSX',3);
omegaSX             = SX.sym('omegaSX',3);
RotSX               = SX.sym('RotSX',9);
TrSX                = SX.sym('TrSX',3);
% Hunt-Crossley contact model
forceSX = HCContactModel(stiffnessSX,radiusSX,dissipationSX,...
    normalSX,transitionVelocitySX,staticFrictionSX,...
    dynamicFrictionSX,viscousFrictionSX,spherePosSX,orFramePosSX,...
    v_linSX,omegaSX,RotSX,TrSX);
f_contactForce = Function('f_contactForce',{stiffnessSX,radiusSX,...
    dissipationSX,normalSX,transitionVelocitySX,staticFrictionSX,...
    dynamicFrictionSX,viscousFrictionSX,spherePosSX,orFramePosSX,...
    v_linSX,omegaSX,RotSX,TrSX},{forceSX});
