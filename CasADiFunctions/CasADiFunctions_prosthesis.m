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
% Function for 8 elements 
etemp8 = SX.sym('etemp8',8);
Jtemp8 = 0;
for i=1:length(etemp8)
    Jtemp8 = Jtemp8 + etemp8(i).^2;
end
Jtemp8 = Jtemp8/8;
f_J8 = Function('f_J8',{etemp8},{Jtemp8});
% Function for 21 elements 
etemp21 = SX.sym('etemp21',21);
Jtemp21 = 0;
for i=1:length(etemp21)
    Jtemp21 = Jtemp21 + etemp21(i).^2;
end
Jtemp21 = Jtemp21/21;
f_J21 = Function('f_J21',{etemp21},{Jtemp21});
% Function for NMuscle_act elements 
etempNMact = SX.sym('etempNMact',NMuscle_act);
JtempNMact = 0;
for i=1:length(etempNMact)
    JtempNMact = JtempNMact + etempNMact(i).^2;
end
JtempNMact = JtempNMact/NMuscle_act;
f_JNMact = Function('f_JNMact',{etempNMact},{JtempNMact});
% Function for 13 elements 
etemp13 = SX.sym('etemp13',13);
Jtemp13 = 0;
for i=1:length(etemp13)
    Jtemp13 = Jtemp13 + etemp13(i).^2;
end
Jtemp13 = Jtemp13/13;
f_J13 = Function('f_J13',{etemp13},{Jtemp13});

%% Sum of squared values (non-normalized)
% Function for 2 elements
etemp2 = SX.sym('etemp2',2);
Jtemp2 = 0;
for i=1:length(etemp2)
    Jtemp2 = Jtemp2 + etemp2(i).^2;
end
f_Jnn2 = Function('f_Jnn2',{etemp2},{Jtemp2});

%% Normalized sum of values to a certain power
% Function for NMuscle_act elements
etempNMactexp   = SX.sym('etempNMactexp',NMuscle_act);
expo            = SX.sym('exp',1);
JtempNMactexp = 0;
for i=1:length(etempNMactexp)
    JtempNMactexp = JtempNMactexp + etempNMactexp(i).^expo;
end
JtempNMactexp = JtempNMactexp/NMuscle_act;
f_JNMactexp = Function('f_JNMactexp',{etempNMactexp,expo},{JtempNMactexp});

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
% Function for 11 elements 
ma_temp11 = SX.sym('ma_temp11',11);
ft_temp11 = SX.sym('ft_temp11',11);
J_sptemp11 = 0;
for i=1:length(ma_temp11)
    J_sptemp11 = J_sptemp11 + ma_temp11(i,1)*ft_temp11(i,1);    
end
f_T11 = Function('f_T11',{ma_temp11,ft_temp11},{J_sptemp11});
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

%% Muscle contraction dynamics
pathmusclemodel = [pathRepo,'/MuscleModel'];
addpath(genpath(pathmusclemodel));
% Function for Hill-equilibrium
FTtilde     = SX.sym('FTtilde',NMuscle_act); % Normalized tendon forces
a           = SX.sym('a',NMuscle_act); % Muscle activations
dFTtilde    = SX.sym('dFTtilde',NMuscle_act); % Time derivative tendon forces
lMT         = SX.sym('lMT',NMuscle_act); % Muscle-tendon lengths
vMT         = SX.sym('vMT',NMuscle_act); % Muscle-tendon velocities
tension_SX  = SX.sym('tension',NMuscle_act); % Tensions
atendon_SX  = SX.sym('atendon',NMuscle_act); % Tendon stiffness
shift_SX    = SX.sym('shift',NMuscle_act); % shift curve    
Hilldiff    = SX(NMuscle_act,1); % Hill-equilibrium
FT          = SX(NMuscle_act,1); % Tendon forces
Fce         = SX(NMuscle_act,1); % Contractile element forces
Fiso        = SX(NMuscle_act,1); % Normalized forces from force-length curve
vMmax       = SX(NMuscle_act,1); % Maximum contraction velocities
massM       = SX(NMuscle_act,1); % Muscle mass
Fpass       = SX(NMuscle_act,1); % Passive element forces
% Parameters of force-length-velocity curves
load Fvparam
load Fpparam
load Faparam
for m = 1:NMuscle_act
    [Hilldiff(m),FT(m),Fce(m),Fpass(m),Fiso(m),vMmax(m),massM(m)] = ...
        ForceEquilibrium_FtildeState_all_tendon(a(m),FTtilde(m),...
        dFTtilde(m),lMT(m),vMT(m),MTparameters_m(:,m),Fvparam,Fpparam,...
        Faparam,tension_SX(m),atendon_SX(m),shift_SX(m));
end
f_forceEquilibrium_FtildeState_all_tendon = ...
    Function('f_forceEquilibrium_FtildeState_all_tendon',{a,FTtilde,...
    dFTtilde,lMT,vMT,tension_SX,atendon_SX,shift_SX},...
    {Hilldiff,FT,Fce,Fpass,Fiso,vMmax,massM});

% Function to get (normalized) muscle fiber lengths
lM      = SX(NMuscle_act,1);
lMtilde = SX(NMuscle_act,1);
for m = 1:NMuscle_act
    [lM(m),lMtilde(m)] = FiberLength_TendonForce_tendon(FTtilde(m),...
        MTparameters_m(:,m),lMT(m),atendon_SX(m),shift_SX(m));
end
f_FiberLength_TendonForce_tendon = Function(...
    'f_FiberLength_Ftilde_tendon',{FTtilde,lMT,atendon_SX,shift_SX},...
    {lM,lMtilde});

% Function to get (normalized) muscle fiber velocities
vM      = SX(NMuscle_act,1);
vMtilde = SX(NMuscle_act,1);
for m = 1:NMuscle_act
    [vM(m),vMtilde(m)] = FiberVelocity_TendonForce_tendon(FTtilde(m),...
        dFTtilde(m),MTparameters_m(:,m),lMT(m),vMT(m),atendon_SX(m),...
        shift_SX(m));
end
f_FiberVelocity_TendonForce_tendon = Function(...
    'f_FiberVelocity_Ftilde_tendon',{FTtilde,dFTtilde,lMT,vMT,...
    atendon_SX,shift_SX},{vM,vMtilde});


%% Passive joint torques
K_pass      = SX.sym('K_pass',4);
theta_pass  = SX.sym('theta_pass',2);
qin_pass    = SX.sym('qin_pass',1);
qdotin_pass = SX.sym('qdotin_pass',1);
% theta_pass 1 and 2 are inverted on purpose.
Tau_pass = K_pass(1,1)*exp(K_pass(2,1)*(qin_pass-theta_pass(2,1))) + ...
    K_pass(3,1)*exp(K_pass(4,1)*(qin_pass-theta_pass(1,1))) ...
    - 0.001*qdotin_pass;
f_PassiveMoments = Function('f_PassiveMoments',{K_pass,theta_pass,...
    qin_pass,qdotin_pass},{Tau_pass});

%% Metabolic energy model
act_SX          = SX.sym('act_SX',NMuscle_act,1); % Muscle activations
exc_SX          = SX.sym('exc_SX',NMuscle_act,1); % Muscle excitations
lMtilde_SX      = SX.sym('lMtilde_SX',NMuscle_act,1); % N muscle fiber lengths
vMtilde_SX      = SX.sym('vMtilde_SX',NMuscle_act,1); % N muscle fiber vel
vM_SX           = SX.sym('vM_SX',NMuscle_act,1); % Muscle fiber velocities
Fce_SX          = SX.sym('FT_SX',NMuscle_act,1); % Contractile element forces
Fpass_SX        = SX.sym('FT_SX',NMuscle_act,1); % Passive element forces
Fiso_SX         = SX.sym('Fiso_SX',NMuscle_act,1); % N forces (F-L curve)
musclemass_SX   = SX.sym('musclemass_SX',NMuscle_act,1); % Muscle mass 
vcemax_SX       = SX.sym('vcemax_SX',NMuscle_act,1); % Max contraction vel
pctst_SX        = SX.sym('pctst_SX',NMuscle_act,1); % Slow twitch ratio 
Fmax_SX         = SX.sym('Fmax_SX',NMuscle_act,1); % Max iso forces
modelmass_SX    = SX.sym('modelmass_SX',1); % Model mass
b_SX            = SX.sym('b_SX',1); % Parameter determining tanh smoothness
% Bhargava et al. (2004)
[energy_total_sm_SX,Adot_sm_SX,Mdot_sm_SX,Sdot_sm_SX,Wdot_sm_SX,...
    energy_model_sm_SX] = getMetabolicEnergySmooth2004all(exc_SX,act_SX,...
    lMtilde_SX,vM_SX,Fce_SX,Fpass_SX,musclemass_SX,pctst_SX,Fiso_SX,...
    Fmax_SX,modelmass_SX,b_SX);
fgetMetabolicEnergySmooth2004all = ...
    Function('fgetMetabolicEnergySmooth2004all',...
    {exc_SX,act_SX,lMtilde_SX,vM_SX,Fce_SX,Fpass_SX,musclemass_SX,...
    pctst_SX,Fiso_SX,Fmax_SX,modelmass_SX,b_SX},{energy_total_sm_SX,...
    Adot_sm_SX,Mdot_sm_SX,Sdot_sm_SX,Wdot_sm_SX,energy_model_sm_SX});
