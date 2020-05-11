clear all

muscles_r = {'hamstrings_r', 'bifemsh_r', 'glut_max_r', 'iliopsoas_r', ...
               'rect_fem_r', 'vasti_r', 'gastroc_r', 'soleus_r', 'tib_ant_r'};
muscles_l = {'hamstrings_l', 'bifemsh_l', 'glut_max_l', 'iliopsoas_l', ...
               'rect_fem_l', 'vasti_l', 'gastroc_l', 'soleus_l', 'tib_ant_l'};
path_2D_model = 'C:\Users\u0101727\Documents\MyRepositories\predictsim_python\OpenSimModel\subject1_2D_mtp\Model\gait12dof18musc_40_FeetRajagopal2D_updated.osim';

params_2D_l = getMTparameters(path_2D_model,muscles_l);
params_2D_r = getMTparameters(path_2D_model,muscles_r);
diff_2D = max(max(abs(params_2D_l-params_2D_r)));

muscles_3D_r = {'bifemlh_r', 'bifemsh_r', 'glut_max2_r', 'psoas_r', ...
               'rect_fem_r', 'vas_int_r', 'med_gas_r', 'soleus_r', 'tib_ant_r'};
muscles_3D_l = {'bifemlh_l', 'bifemsh_l', 'glut_max2_l', 'psoas_l', ...
               'rect_fem_l', 'vas_int_l', 'med_gas_l', 'soleus_l', 'tib_ant_l'};
path_3D_model = 'C:\Users\u0101727\Documents\MyRepositories\predictsim_python\OpenSimModel\subject1_HamnerModel\Model\HamnerModel_FeetRajagopal.osim';
params_3D_l = getMTparameters(path_3D_model,muscles_3D_l);
params_3D_r = getMTparameters(path_3D_model,muscles_3D_r);
diff_3D = max(max(abs(params_3D_l-params_3D_r)));
diff_2D_3D = max(max(abs(params_2D_l(2:end,:)-params_3D_r(2:end,:))));


% trials = {'14', '15', '23', '25', '27', '60', '61', '63', '64', '65'};
% 
% figure()
% for i = 1:length(trials)
%     pathGRF = ['C:\Users\u0101727\Documents\MyRepositories\predictsim_python\OpenSimModel\GRF\GRF_gait_', trials{i}, '.mot'];
%     GRF = getGRF_temp(pathGRF);
%     
%     values.GRFmax(i,:) = max([GRF.val.l;GRF.val.r]);
%     values.GRFmin(i,:) = min([GRF.val.l;GRF.val.r]);
%     
%     values.GRMmax(i,:) = max([GRF.MorGF.l;GRF.MorGF.r]);
%     values.GRMmin(i,:) = min([GRF.MorGF.l;GRF.MorGF.r]);
%     
%     subplot(4,3,1)
%     plot(GRF.val.l())
%     
% end
% 
% values.GRFmaxmax = max(values.GRFmax);
% values.GRFminmin = min(values.GRFmin);
% values.GRMmaxmax = max(values.GRMmax);
% values.GRMminmin = min(values.GRMmin);
