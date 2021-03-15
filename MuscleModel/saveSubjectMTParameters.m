% This script extracts the muscle-tendon parameters of the subject based on
% the provided model and saves them in a structore for use in the
% simulations.
%
% Author: Antoine Falisse
%
% Data: 25/3/2020

clear all
close all
clc

%% User inputs
subjectName = 'subject2'; % subject name.
modelName = 'subject2_withMTP_scaled'; % name of the osim file.
outputFileName = 'subject2_mtpPin';
% Muscles for which we want to extract the muscle-tendon parameters.
muscleNames = {'glut_med1_r','glut_med2_r','glut_med3_r',...
        'glut_min1_r','glut_min2_r','glut_min3_r','semimem_r',...
        'semiten_r','bifemlh_r','bifemsh_r','sar_r','add_long_r',...
        'add_brev_r','add_mag1_r','add_mag2_r','add_mag3_r','tfl_r',...
        'pect_r','grac_r','glut_max1_r','glut_max2_r','glut_max3_r',......
        'iliacus_r','psoas_r','quad_fem_r','gem_r','peri_r',...
        'rect_fem_r','vas_med_r','vas_int_r','vas_lat_r','med_gas_r',...
        'lat_gas_r','soleus_r','tib_post_r','flex_dig_r','flex_hal_r',...
        'tib_ant_r','per_brev_r','per_long_r','per_tert_r','ext_dig_r',...
        'ext_hal_r','ercspn_r','intobl_r','extobl_r','ercspn_l',...
        'intobl_l','extobl_l'};

%% Extract and save model muscle-tendon parameters
pathmain = pwd;
[pathRepo,~,~] = fileparts(pathmain);
pathModel = [pathRepo,'/OpenSimModel/',subjectName,'/Models/',modelName,'.osim'];
pathVariousFunctions = [pathRepo,'/VariousFunctions'];
addpath(genpath(pathVariousFunctions));
MTparameters = getMTparameters(pathModel, muscleNames);
save(['MTparameters_', outputFileName], 'MTparameters');
