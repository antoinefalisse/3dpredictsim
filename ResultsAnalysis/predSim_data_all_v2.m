% This file contains the structure used to save the results of the
% predictive simulations and is loaded in the different files that are 
% processing and analyzing the results

v_tgt       = settings(ww(k),1);    % average speed
tol_ipopt   = settings(ww(k),2);    % tolerance (means 1e-(tol_ipopt))
N           = settings(ww(k),3);    % number of mesh intervals
W.E         = settings(ww(k),4);    % weight metabolic energy
W.Ak        = settings(ww(k),5);    % weight joint accelerations
W.ArmE      = settings(ww(k),6);    % weight arm excitations
W.passMom   = settings(ww(k),7);    % weight passive torques
W.A         = settings(ww(k),8);    % weight muscle activations
exp_E       = settings(ww(k),9);    % power metabolic energy
IGsel       = settings(ww(k),10);   % initial guess identifier
cm          = settings(ww(k),11);   % contact model identifier
IGm         = settings(ww(k),12);   % initial guess mode identifier
IGcase      = settings(ww(k),13);   % initial guess case identifier
h_weak      = settings(ww(k),14);   % weakness hip actuators
vMax_s      = settings(ww(k),15);   % maximal contraction velocity identifier
pf_weak     = settings(ww(k),16);   % weakness ankle plantaflexors
mE          = settings(ww(k),17);   % metabolic energy model identifier
coCont      = settings(ww(k),18);   % co-contraction identifier

% load results
pathmain = pwd;
[pathrepo,~,~] = fileparts(pathmain);
pathresults = [pathrepo,'/Results/PredSim_all_v2'];
load([pathresults,'/Results_all_v2.mat']);
v_tgt_id = round(v_tgt,2);

Qs_opt(ww(k)).m = Results_all.(['Case_',num2str(ww(k))]).Qs_opt;
Qdots_opt(ww(k)).m = Results_all.(['Case_',num2str(ww(k))]).Qdots_opt;
Acts_opt(ww(k)).m = Results_all.(['Case_',num2str(ww(k))]).Acts_opt;
Ts_opt(ww(k)).m = Results_all.(['Case_',num2str(ww(k))]).Ts_opt;
GRFs_opt(ww(k)).m = Results_all.(['Case_',num2str(ww(k))]).GRFs_opt;
COT_opt(ww(k)).m = Results_all.(['Case_',num2str(ww(k))]).COT_opt;
StrideLength_opt(ww(k)).m = Results_all.(['Case_',num2str(ww(k))]).StrideLength_opt;
StepWidth_opt(ww(k)).m = Results_all.(['Case_',num2str(ww(k))]).StepWidth_opt;
Cost(ww(k)).m = Results_all.(['Case_',num2str(ww(k))]).stats.iterations.obj(end);
t_proc_solver =  Results_all.(['Case_',num2str(ww(k))]).stats.t_proc_nlp_f + ...
    Results_all.(['Case_',num2str(ww(k))]).stats.t_proc_nlp_g + ...
    Results_all.(['Case_',num2str(ww(k))]).stats.t_proc_nlp_grad + ...
    Results_all.(['Case_',num2str(ww(k))]).stats.t_proc_nlp_grad_f + ...
    Results_all.(['Case_',num2str(ww(k))]).stats.t_proc_nlp_jac_g;
t_proc_nlp = Results_all.(['Case_',num2str(ww(k))]).stats.t_proc_total - ...
    t_proc_solver;
CPU_NLP(ww(k)).m = t_proc_nlp;
CPU_IPOPT(ww(k)).m = t_proc_solver;
    