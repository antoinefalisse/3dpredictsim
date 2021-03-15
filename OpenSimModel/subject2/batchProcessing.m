clear all
close all
clc

batchIK = 0;
batchEL = 0;
batchID = 1;

%% User setting
useKS = 0; % set to 1 to use the Kalman Smoother
modelNames = {'subject2_withMTP_weldRadius_scaled_FK'};
% modelNames = {'subject1_Maarten_withMTP_scaled'};
% modelNames = {'subject1_Antoine_withMTP_scaled'};
modelName = modelNames{1};
subjects = {'subject1'}; % fixed for now.
pathMain = pwd;
pathModels = [pathMain,'\Models\'];
pathIK = [pathMain,'\IK\'];
pathIKOut = [pathMain,'\IK\'];
if ~(exist(pathIKOut,'dir')==7)
    mkdir(pathIKOut);
end
pathTRC = [pathMain,'\TRC\'];
pathMain = pwd;
pathEL = [pathMain,'\ExternalLoads\'];
pathGRF = [pathMain,'\GRF\'];
pathID = [pathMain,'\ID\'];
% pathIK = ['IK\'];
% pathIKOut = ['IK\',modelName];
% pathTRC = ['TRC\'];

%% Fixed settings
trials.subject1.gait_14.startTime = 3.05;
trials.subject1.gait_14.endTime = 4.5;

trials.subject1.gait_15.startTime = 2;
trials.subject1.gait_15.endTime = 3.4;

trials.subject1.gait_23.startTime = 1.2;
trials.subject1.gait_23.endTime = 2.6;

trials.subject1.gait_25.startTime = 0.95;
trials.subject1.gait_25.endTime = 2.4;

trials.subject1.gait_27.startTime = 1.75;
trials.subject1.gait_27.endTime = 3.1;

trials.subject1.gait_60.startTime = 1.1;
trials.subject1.gait_60.endTime = 2.45;

trials.subject1.gait_61.startTime = 1.75;
trials.subject1.gait_61.endTime = 3;

trials.subject1.gait_63.startTime = 0.8;
trials.subject1.gait_63.endTime = 2;

trials.subject1.gait_64.startTime = 1;
trials.subject1.gait_64.endTime = 2.4;

trials.subject1.gait_65.startTime = 0.9;
trials.subject1.gait_65.endTime = 2.3;

trials.subject1.gait_67.startTime = 1.25;
trials.subject1.gait_67.endTime = 2.6;

%% Batch IK
import org.opensim.modeling.*
if batchIK
    for i = 1:length(modelNames)  
        c_modelName = [modelNames{i}, '.osim'];
        modelFile = [pathModels, c_modelName];
        model = Model([pathModels, c_modelName]);
        model.initSystem();
        trialNames = fieldnames(trials.(subjects{i}));
        for j = 1:length(trialNames)
            c_trial = trialNames{j};
            % Generic setup IK.
            genericSetupIK = 'SetupIK_generic.xml';
            ikTool = InverseKinematicsTool([pathIKOut, genericSetupIK]);
            % Set name
            ikTool.setName(c_trial);
            ikTool.set_model_file([pathModels, modelName, '.osim'])
            % Set results directory
            ikTool.setResultsDir(pathIKOut);
            ikTool.setOutputMotionFileName([pathIKOut, 'IK_', c_trial, '.mot']);
            % Set model file
    %         ikTool.setModel(model);
            % Set marker file
            ikTool.setMarkerDataFileName([pathTRC, c_trial, '.trc']);
            % Time
            ikTool.setStartTime(trials.(subjects{i}).(c_trial).startTime);
            ikTool.setEndTime(trials.(subjects{i}).(c_trial).endTime)           
            % Save setup file
            PathSetupIK = [pathIKOut, 'SetupIK_', c_trial, '.xml'];
            ikTool.print(PathSetupIK);         
            % Run IK
            if useKS
                Command = ['KS ', '-S "', PathSetupIK '"'];
            else
                Command = ['opensim-cmd run-tool "' PathSetupIK '"'];
            end
            system(Command);
        end
    end
end

%% Batch external Loads
if batchEL
    for i = 1:length(modelNames)  
        % Model
        c_modelName = [modelNames{i}, '.osim'];
        modelFile = [pathModels, c_modelName];
        model = Model([pathModels, c_modelName]);
        model.initSystem();
        trialNames = fieldnames(trials.(subjects{i}));
        for j = 1:length(trialNames)
            c_trial = trialNames{j};
            % Generic ExternalLoads files
            genericSetupForEL = 'ExternalLoads_overground_gait.xml';
            ELTool = ExternalLoads([pathEL genericSetupForEL], true);
            ELTool.setDataFileName([pathGRF,'GRF_',c_trial,'.mot']);
            ELTool.setName(c_trial);
            SetupForEL = ['External_Loads_',c_trial,'.xml'];
            ELTool.print([pathEL,SetupForEL]); 
        end
    end
end

%% Batch ID
if batchID
    for i = 1:length(modelNames) 
        % Model
        c_modelName = [modelNames{i}, '.osim'];
        modelFile = [pathModels, c_modelName];
        model = Model([pathModels, c_modelName]);
        model.initSystem();
        trialNames = fieldnames(trials.(subjects{i}));
        for j = 1:length(trialNames)
            c_trial = trialNames{j};
            % Generic ID files
            genericSetupForID = 'SetupID_gait_overground_generic.xml';
            idTool = InverseDynamicsTool([pathID genericSetupForID]);
            % Set model
            idTool.setModel(model);
            idTool.setModelFileName(modelFile);
            idTool.setName(c_trial);
            % time
            idTool.setStartTime(trials.(subjects{i}).(c_trial).startTime);
            idTool.setEndTime(trials.(subjects{i}).(c_trial).endTime)        
            % External Loads and IK
            SetupForEL = ['External_Loads_',c_trial,'.xml'];
            ExternalLoadspath = [pathEL,SetupForEL];
            idTool.setExternalLoadsFileName(ExternalLoadspath)
            IKpath = [pathIKOut,'IK_',c_trial,'.mot'];
            idTool.setCoordinatesFileName(IKpath)
            % Cutoff frequency
            idTool.setLowpassCutoffFrequency(6);
            % Where to save results
            idTool.setResultsDir(pathID);
            idTool.setOutputGenForceFileName(['ID_',c_trial,'.sto']);
            % Save setup file
            PathSetupID = [pathID,'SetupID_',c_trial,'.xml'];        
            idTool.print(PathSetupID);         
            % Run ID
            Command = ['ID' ' -S ' PathSetupID];
            system(Command);
        end
    end
end
