% We analyze the experimental data to provide reference data for the
% comparison with the results of the predictive simulations.
% We average considering heel strike on the right leg, we therefore adjust
% the data with heel strike on the left leg.
% We take data from a full gait cycle starting at heel strike on the force
% plate until the first initial contact of the same other leg with the
% ground that we visually extract from OpenSim.

clear all
close all
clc

plotGRF = 1;
plotGRM = 0;
plotKinematics = 1;
plotKinematicsSpeedsAcc = 0;
plotKinetics = 1;
plotEMG = 0;
saveBounds = 0;
saveIG = 0;
createMotion = 0;
createGRF = 0;
saveExperimentalData = 0;

% modelNames = {'subject1_Friedl_withMTP_scaled'};
modelNames = {'subject2_withMTP_weldRadius_scaled_FK'};
% modelNames = {'subject1_Antoine_withMTP_scaled'};
modelName = modelNames{1};
subjects = {'subject1'};
body_mass = 62; % fixed for now.
body_weight = body_mass*9.81;

% timeICoff indicates the first initial contact after the force plate, this
% is visually extracted from OpenSim
% trials.subject1.gait_14.icOff = 4.27;
% trials.subject1.gait_15.icOff = 3.2;
% trials.subject1.gait_23.icOff = 2.41;
% trials.subject1.gait_25.icOff = 2.2;
% trials.subject1.gait_27.icOff = 2.92;
% trials.subject1.gait_60.icOff = 2.26;
trials.subject1.gait_61.icOff = 2.87;
% trials.subject1.gait_63.icOff = 1.86;
trials.subject1.gait_64.icOff = 2.14;
% trials.subject1.gait_65.icOff = 2.1;
% trials.subject1.gait_67.icOff = 2.38;

pathMain = pwd;
[pathOpenSimModel,~,~] = fileparts(pathMain);
[pathRepo,~,~] = fileparts(pathOpenSimModel);
pathGRF = [pathMain,'\GRF\'];
pathIK = [pathMain,'\IK\'];
pathID = [pathMain,'\ID\'];
pathAnalysis = [pathMain,'\Analysis\'];

% Kinematics
joints = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx',...
    'pelvis_ty','pelvis_tz','hip_flexion_l','hip_adduction_l',...
    'hip_rotation_l','hip_flexion_r','hip_adduction_r','hip_rotation_r',...
    'knee_angle_l','knee_angle_r','ankle_angle_l','ankle_angle_r',...
    'subtalar_angle_l','subtalar_angle_r',...
    'mtp_angle_l','mtp_angle_r',...
    'lumbar_extension','lumbar_bending','lumbar_rotation',...
    'arm_flex_l','arm_add_l','arm_rot_l',...
    'arm_flex_r','arm_add_r','arm_rot_r',...
    'elbow_flex_l','elbow_flex_r'};
joints_oneleg = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx',...
    'pelvis_ty','pelvis_tz','hip_flexion_','hip_adduction_',...
    'hip_rotation_','knee_angle_','ankle_angle_','subtalar_angle_',...
    'mtp_angle_',...
    'lumbar_extension','lumbar_bending','lumbar_rotation',...
    'arm_flex_','arm_add_','arm_rot_','elbow_flex_'};
joints_onelegb = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx',...
    'pelvis_ty','pelvis_tz','hip_flexion','hip_adduction',...
    'hip_rotation','knee_angle','ankle_angle','subtalar_angle',...
    'mtp_angle',...
    'lumbar_extension','lumbar_bending','lumbar_rotation',...
    'arm_flex','arm_add','arm_rot','elbow_flex'};
joints_rleg = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx',...
    'pelvis_ty','pelvis_tz','hip_flexion_r','hip_adduction_r',...
    'hip_rotation_r','knee_angle_r','ankle_angle_r','subtalar_angle_r',...
    'mtp_angle_r',...
    'lumbar_extension','lumbar_bending','lumbar_rotation',...
    'arm_flex_r','arm_add_r','arm_rot_r','elbow_flex_r'};
joints_s = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx',...
    'pelvis_ty','pelvis_tz','lumbar_extension','lumbar_bending',...
    'lumbar_rotation'};
joints_sopp = {'pelvis_list','pelvis_rotation','pelvis_tz',...
    'lumbar_bending','lumbar_rotation'};
channels = {'HamL_r','TA_r','PerL_r','GL_r','HamM_r','Sol_r','VL_r',...
    'VM_r','PerB_l','TA_l','PerL_l','GL_l','GM_l','Sol_l','VL_l','VM_l',...
    'AddL_l','RF_l','TFL_l','GluMed_l','HamL_l','HamM_l','GluMed_r',...
    'RF_r'};
channels_r = {'HamL_r','TA_r','PerL_r','GL_r','HamM_r','Sol_r','VL_r',...
    'VM_r','GluMed_r','RF_r'};
channels_l = {'HamL_l','TA_l','PerL_l','GL_l','HamM_l','Sol_l','VL_l',...
    'VM_l','GluMed_l','RF_l','PerB_l','GM_l','AddL_l','TFL_l'};

channels_all = {'HamL','TA','PerL','GL','HamM','Sol','VL',...
    'VM','GluMed','RF','PerB','GM','AddL','TFL'};

% trials for which on channels 6 and 7 are valid for the right leg
trials_v67 = {'gait_63','gait_67','gait_65'};
% trials for which on channels 3, 5, 6 and 7 are valid for the right leg
trials_v3567 = {'gait_15','gait_27'};

for i = 1:length(joints_rleg)
    idx_r_in_all(i) = find(strcmp(joints_rleg{i},joints));
end

jointi.pelvis.tilt  = 1; 
jointi.pelvis.list  = 2; 
jointi.pelvis.rot   = 3; 
jointi.pelvis.tx    = 4;
jointi.pelvis.ty    = 5;
jointi.pelvis.tz    = 6;
jointi.hip_flex.l   = 7;
jointi.hip_add.l    = 8;
jointi.hip_rot.l    = 9;
jointi.hip_flex.r   = 10;
jointi.hip_add.r    = 11;
jointi.hip_rot.r    = 12;
jointi.knee.l       = 13;
jointi.knee.r       = 14;
jointi.ankle.l      = 15;
jointi.ankle.r      = 16;
jointi.subt.l       = 17;
jointi.subt.r       = 18;
jointi.mtp.l        = 19;
jointi.mtp.r        = 20;
jointi.trunk.ext    = 21;
jointi.trunk.ben    = 22;
jointi.trunk.rot    = 23;
jointi.sh_flex.l    = 24;
jointi.sh_add.l     = 25;
jointi.sh_rot.l     = 26;
jointi.sh_flex.r    = 27;
jointi.sh_add.r     = 28;
jointi.sh_rot.r     = 29;
jointi.elb.l        = 30;
jointi.elb.r        = 31;
QsSymA_ptx = [jointi.pelvis.tilt,jointi.pelvis.tx,...
    jointi.pelvis.ty,...
    jointi.hip_flex.l:jointi.trunk.ext,...
    jointi.sh_flex.l:jointi.elb.r];
QsSymB_ptx = [jointi.pelvis.tilt,jointi.pelvis.tx,...
    jointi.pelvis.ty,...    
    jointi.hip_flex.r:jointi.hip_rot.r,...
    jointi.hip_flex.l:jointi.hip_rot.l,...
    jointi.knee.r,jointi.knee.l,...
    jointi.ankle.r,jointi.ankle.l,...
    jointi.subt.r,jointi.subt.l,...
    jointi.mtp.r,jointi.mtp.l,...
    jointi.trunk.ext,...
    jointi.sh_flex.r:jointi.sh_rot.r,...
    jointi.sh_flex.l:jointi.sh_rot.l,...
    jointi.elb.r,jointi.elb.l]; 
QsOpp = [jointi.pelvis.list:jointi.pelvis.rot,jointi.pelvis.tz,...
        jointi.trunk.ben:jointi.trunk.rot];
    
threshold = 20; % threshold to detect contact
N = 100; % number of time points for interpolation
pathVariousFunctions = [pathRepo,'/VariousFunctions'];
addpath(genpath(pathVariousFunctions));

% Timing initial contacts & GRFs
for i = 1:length(modelNames)
    count = 1;
    trialNames = fieldnames(trials.(subjects{i}));
    for j = 1:length(trialNames)
        c_trial = trialNames{j};
        
        %% GRF
        pathGRFFile = [pathGRF, 'GRF_', c_trial, '.mot'];        
        GRF = getGRF(pathGRFFile);        
        [time_IC.(subjects{i}){count},idx_IC.(subjects{i}){count},...
            leg1.(subjects{i}){count}] = getIC_1FP(GRF,threshold,trials.(subjects{i}).(c_trial).icOff);         
        if strcmp(leg1.(subjects{i}){count},'r')
            GRFsel.(subjects{i}){count} = GRF.val.all(idx_IC.(subjects{i}){count}(1):idx_IC.(subjects{i}){count}(2),:);
            GRMsel.(subjects{i}){count} = GRF.MorGF.all(idx_IC.(subjects{i}){count}(1):idx_IC.(subjects{i}){count}(2),:);
        else
            % if left then we invert right and left and also take the
            % opposite for the z-axis
            GRFsel.(subjects{i}){count} = GRF.val.all(idx_IC.(subjects{i}){count}(1):idx_IC.(subjects{i}){count}(2),[1,5:7,2:4]);
            GRFsel.(subjects{i}){count}(:,[4,7]) = -GRFsel.(subjects{i}){count}(:,[4,7]);  
            
            GRMsel.(subjects{i}){count} = GRF.MorGF.all(idx_IC.(subjects{i}){count}(1):idx_IC.(subjects{i}){count}(2),[1,5:7,2:4]);
            GRMsel.(subjects{i}){count}(:,[4,7]) = -GRMsel.(subjects{i}){count}(:,[4,7]);
        end
        % interpolation over N points
        step = (GRFsel.(subjects{i}){count}(end,1)-GRFsel.(subjects{i}){count}(1,1))/(N-1);
        interval.(subjects{i}){count} = GRFsel.(subjects{i}){count}(1,1):step:GRFsel.(subjects{i}){count}(end,1);
        GRFinterp.(subjects{i})(:,:,count) = interp1(GRFsel.(subjects{i}){count}(:,1),GRFsel.(subjects{i}){count}(:,2:end),interval.(subjects{i}){count});
        GRMinterp.(subjects{i})(:,:,count) = interp1(GRMsel.(subjects{i}){count}(:,1),GRMsel.(subjects{i}){count}(:,2:end),interval.(subjects{i}){count});
        % Only select the right leg
        GRFinterpR.(subjects{i}) = GRFinterp.(subjects{i})(:,1:3,:);
        GRFinterpR.(subjects{i}) = GRFinterpR.(subjects{i})./(body_weight/100);
        
        GRMinterpR.(subjects{i}) = GRMinterp.(subjects{i})(:,1:3,:);
        GRMinterpR.(subjects{i}) = GRMinterpR.(subjects{i})./(body_weight/100);
                
        %% Kinematics
        % Coordinate values
        pathIKFile = [pathIK, 'IK_', c_trial, '.mot'];
        Qs = getIK(pathIKFile, joints);            
        if strcmp(leg1.(subjects{i}){count},'r')        
            IKsel.(subjects{i}){count} = Qs.allfilt;
        else
            % if left then we invert right and left and also take the
            % opposite for rotations around x and y axes
            IKsel.(subjects{i}){count}(:,[1,QsSymA_ptx+1]) = Qs.allfilt(:,[1,QsSymB_ptx+1]);
            IKsel.(subjects{i}){count}(:,QsOpp+1) = -Qs.allfilt(:,QsOpp+1);            
        end
        IKinterp.(subjects{i})(:,:,count) = interp1(IKsel.(subjects{i}){count}(:,1),IKsel.(subjects{i}){count}(:,2:end),interval.(subjects{i}){count});                  
        % Adjust pelvis tx to start from 0       
        IKinterp.(subjects{i})(:,jointi.pelvis.tx,count) = IKinterp.(subjects{i})(:,jointi.pelvis.tx,count) - IKinterp.(subjects{i})(1,jointi.pelvis.tx,count);   
        IKinterp.(subjects{i})(:,[1:3,7:end],count) = IKinterp.(subjects{i})(:,[1:3,7:end],count)*180/pi;
        % Only select the right leg
        IKinterpR.(subjects{i}) = IKinterp.(subjects{i})(:,idx_r_in_all,:);
        
        % Coordinate speeds and accelerations
        % Spline approximation
        for ii = 1:size(IKinterpR.(subjects{i})(:,:,count),2)
            Qss.datafiltspline(ii) = ...
                spline(interval.(subjects{i}){count}',...
                IKinterpR.(subjects{i})(:,ii,count));            
            [Qs_spline.(subjects{i})(:,ii,count),...
                Qdots_spline.(subjects{i})(:,ii,count),...
                Qdotdots_spline.(subjects{i})(:,ii,count)] = ...
                SplineEval_ppuval(Qss.datafiltspline(ii),...
                interval.(subjects{i}){count}',1);
        end       
        
        % Kinetics
        pathIDFile = [pathID,'ID_',c_trial,'.sto'];
        ID = getID(pathIDFile,joints); 
        ID_s.all = ID.all(:,1);
        if strcmp(leg1.(subjects{i}){count},'r') 
            idleg = 'r';
        else
            idleg = 'l';
        end
        for jj = 1:length(joints_oneleg)
            if find(strcmp(joints_s,joints_oneleg{jj}))
                ID_s.all(:,jj+1) = ID.(joints_oneleg{jj});                
            else
                ID_s.all(:,jj+1) = ID.([joints_oneleg{jj},idleg]);
            end
            if find(strcmp(joints_sopp,joints_oneleg{jj})) & strcmp(leg1.(subjects{i}){count},'l') 
                ID_s.all(:,jj+1) = -ID_s.all(:,jj+1);
            end
        end        
        IDinterp.(subjects{i})(:,:,count) = interp1(ID_s.all(:,1),ID_s.all(:,2:end),interval.(subjects{i}){count}); 
%         
%         % EMG
%         pathEMG = [pathData,'\EMG\EMG_',trials.(subjects{i}){j}];
%         load(pathEMG,'EMG');
%         EMG_s.all = EMG.data(:,1);
%         if strcmp(leg1.(subjects{i}){count},'r') 
%             % For some trials, EMGs are not good and are therefore
%             % removed
%             if find(strcmp(trials.(subjects{i}){j},trials_v67))
%                 for jj = 1:length(channels_r)                                 
%                     EMG_s.all(:,jj+1) = EMG.data(:,strcmp(channels_r{jj},EMG.colheaders));
%                     if ~(jj==6 || jj==7)
%                         EMG_s.all(:,jj+1) = nan;
%                     end
%                 end                
%             elseif find(strcmp(trials.(subjects{i}){j},trials_v3567))
%                 for jj = 1:length(channels_r)  
%                     EMG_s.all(:,jj+1) = EMG.data(:,strcmp(channels_r{jj},EMG.colheaders));
%                     if ~(jj==3 || jj==5 || jj==6 || jj==7)
%                         EMG_s.all(:,jj+1) = nan;
%                     end
%                 end
%             else                     
%                 for jj = 1:length(channels_r)                                 
%                     EMG_s.all(:,jj+1) = EMG.data(:,strcmp(channels_r{jj},EMG.colheaders));
%                 end
%             end
%             % For the right leg, there are 4 muscles fewer so we use nan
%             EMG_s.all = [EMG_s.all,nan(size(EMG_s.all,1),4)];
%         else
%             for jj = 1:length(channels_l)
%                 EMG_s.all(:,jj+1) = EMG.data(:,strcmp(channels_l{jj},EMG.colheaders));
%             end
%         end
%         EMGinterp.(subjects{i})(:,:,count) = interp1(EMG_s.all(:,1),EMG_s.all(:,2:end),interval.(subjects{i}){count});            

        time_ICall.(subjects{i})(j,:) = time_IC.(subjects{i}){count};
        count =  count + 1;   
    end 
    GRFref.(subjects{i}).mean = mean(GRFinterpR.(subjects{i}),3);
    GRFref.(subjects{i}).std = std(GRFinterpR.(subjects{i}),[],3);
    GRMref.(subjects{i}).mean = mean(GRMinterpR.(subjects{i}),3);
    GRMref.(subjects{i}).std = std(GRMinterpR.(subjects{i}),[],3);
    Qref.(subjects{i}).Qs.mean = mean(Qs_spline.(subjects{i}),3);
    Qref.(subjects{i}).Qs.std = std(Qs_spline.(subjects{i}),[],3);
    Qref.(subjects{i}).Qdots.mean = mean(Qdots_spline.(subjects{i}),3);
    Qref.(subjects{i}).Qdots.std = std(Qdots_spline.(subjects{i}),[],3);
    Qref.(subjects{i}).Qdotdots.mean = mean(Qdotdots_spline.(subjects{i}),3);
    Qref.(subjects{i}).Qdotdots.std = std(Qdotdots_spline.(subjects{i}),[],3);
    Qref.(subjects{i}).colheaders = joints_onelegb;
    IDref.(subjects{i}).mean = mean(IDinterp.(subjects{i}),3);
    IDref.(subjects{i}).std = std(IDinterp.(subjects{i}),[],3);
    IDref.(subjects{i}).colheaders = joints_onelegb;
%     EMGref.(subjects{i}).all = EMGinterp.(subjects{i});
%     EMGref.(subjects{i}).mean = nanmean(EMGinterp.(subjects{i}),3);
%     EMGref.(subjects{i}).std = nanstd(EMGinterp.(subjects{i}),[],3);
%     EMGref.(subjects{i}).colheaders = channels_all;
%     
%     % Joint power
%     power.(subjects{i}) = Qdots_spline.(subjects{i}).*IDinterp.(subjects{i}).*pi/180;    
%     Pref.(subjects{i}).mean = nanmean(power.(subjects{i}),3);
%     Pref.(subjects{i}).std = nanstd(power.(subjects{i}),[],3);
%     Pref.(subjects{i}).colheaders = joints_onelegb;
%     
    if saveExperimentalData
%         pathReferenceData = [pathRepo,'\ReferenceData\',subjects{i},'\'];
%         save([pathReferenceData,'GRFref_mtp'],'GRFref');
        save([pathAnalysis,'Qref_',modelName],'Qref');
%         save([pathReferenceData,'IDref_mtp'],'IDref');
%         save([pathReferenceData,'EMGref_mtp'],'EMGref');
%         save([pathReferenceData,'Pref_mtp'],'Pref');
    end       

    if plotGRF
        figure()
        for c = 1:size(GRFinterpR.(subjects{i}),3)
            plot(GRFinterpR.(subjects{i})(:,1,c),'b','linewidth',0.5);
            hold on;
            plot(GRFinterpR.(subjects{i})(:,2,c),'r','linewidth',0.5);
            plot(GRFinterpR.(subjects{i})(:,3,c),'m','linewidth',0.5);        
        end
        plot(GRFref.(subjects{i}).mean(:,1),'b','linewidth',2);
        plot(GRFref.(subjects{i}).mean(:,1)+GRFref.(subjects{i}).std(:,1),'b--','linewidth',2);
        plot(GRFref.(subjects{i}).mean(:,1)-GRFref.(subjects{i}).std(:,1),'b--','linewidth',2);
        plot(GRFref.(subjects{i}).mean(:,2),'r','linewidth',2);
        plot(GRFref.(subjects{i}).mean(:,2)+GRFref.(subjects{i}).std(:,2),'r--','linewidth',2);
        plot(GRFref.(subjects{i}).mean(:,2)-GRFref.(subjects{i}).std(:,2),'r--','linewidth',2);
        plot(GRFref.(subjects{i}).mean(:,3),'m','linewidth',2);
        plot(GRFref.(subjects{i}).mean(:,3)+GRFref.(subjects{i}).std(:,3),'m--','linewidth',2);
        plot(GRFref.(subjects{i}).mean(:,3)-GRFref.(subjects{i}).std(:,3),'m--','linewidth',2)
        title('GRFs','Fontsize',20);        
    end
    
    if plotGRM
        figure()
        for c = 1:size(GRMinterpR.(subjects{i}),3)
            plot(GRMinterpR.(subjects{i})(:,1,c),'b','linewidth',0.5);
            hold on;
            plot(GRMinterpR.(subjects{i})(:,2,c),'r','linewidth',0.5);
            plot(GRMinterpR.(subjects{i})(:,3,c),'m','linewidth',0.5);        
        end
        plot(GRMref.(subjects{i}).mean(:,1),'b','linewidth',2);
        plot(GRMref.(subjects{i}).mean(:,1)+GRMref.(subjects{i}).std(:,1),'b--','linewidth',2);
        plot(GRMref.(subjects{i}).mean(:,1)-GRMref.(subjects{i}).std(:,1),'b--','linewidth',2);
        plot(GRMref.(subjects{i}).mean(:,2),'r','linewidth',2);
        plot(GRMref.(subjects{i}).mean(:,2)+GRMref.(subjects{i}).std(:,2),'r--','linewidth',2);
        plot(GRMref.(subjects{i}).mean(:,2)-GRMref.(subjects{i}).std(:,2),'r--','linewidth',2);
        plot(GRMref.(subjects{i}).mean(:,3),'m','linewidth',2);
        plot(GRMref.(subjects{i}).mean(:,3)+GRMref.(subjects{i}).std(:,3),'m--','linewidth',2);
        plot(GRMref.(subjects{i}).mean(:,3)-GRMref.(subjects{i}).std(:,3),'m--','linewidth',2)
        title('GRMs','Fontsize',20);        
    end

    if plotKinematics
        coordinatenames = {'pelvis-tilt','pelvis-list','pelvis-rotation','pelvis-tx',...
            'pelvis-ty','pelvis-tz','hip-flexion','hip-adduction',...
            'hip-rotation','knee-angle','ankle-angle','subtalar-angle',...
            'mtp-angle',...
            'lumbar-extension','lumbar-bending','lumbar-rotation',...
            'arm-flex','arm-add','arm-rot','elbow-flex'};
        
        
        figure()
        for jj = 1:size(Qref.(subjects{i}).Qs.mean,2)
            subplot(5,4,jj)
            for c = 1:size(IKinterpR.(subjects{i}),3)
                plot(Qs_spline.(subjects{i})(:,jj,c),'k','linewidth',0.5); hold on
            end
            plot(Qref.(subjects{i}).Qs.mean(:,jj),'k','linewidth',2); hold on
            plot(Qref.(subjects{i}).Qs.mean(:,jj)+Qref.(subjects{i}).Qs.std(:,jj),'k--','linewidth',2);
            plot(Qref.(subjects{i}).Qs.mean(:,jj)-Qref.(subjects{i}).Qs.std(:,jj),'k--','linewidth',2);
            title(coordinatenames{jj},'Fontsize', 16)
        end  
        s = sgtitle('Coordinate values');
        set(s,'Fontsize',20);
        
        if plotKinematicsSpeedsAcc
            figure()
            for jj = 1:size(Qref.(subjects{i}).Qdots.mean,2)
                subplot(5,4,jj)
                for c = 1:size(IKinterpR.(subjects{i}),3)
                    plot(Qdots_spline.(subjects{i})(:,jj,c),'k','linewidth',0.5); hold on
                end
                plot(Qref.(subjects{i}).Qdots.mean(:,jj),'k','linewidth',2); hold on
                plot(Qref.(subjects{i}).Qdots.mean(:,jj)+Qref.(subjects{i}).Qdots.std(:,jj),'k--','linewidth',2);
                plot(Qref.(subjects{i}).Qdots.mean(:,jj)-Qref.(subjects{i}).Qdots.std(:,jj),'k--','linewidth',2);
                title(coordinatenames{jj},'Fontsize', 16)
            end  
            s = sgtitle('Coordinate speeds');
            set(s,'Fontsize',20);

            figure()
            for jj = 1:size(Qref.(subjects{i}).Qdotdots.mean,2)
                subplot(5,4,jj)
                for c = 1:size(IKinterpR.(subjects{i}),3)
                    plot(Qdotdots_spline.(subjects{i})(:,jj,c),'k','linewidth',0.5); hold on
                end
                plot(Qref.(subjects{i}).Qdotdots.mean(:,jj),'k','linewidth',2); hold on
                plot(Qref.(subjects{i}).Qdotdots.mean(:,jj)+Qref.(subjects{i}).Qdotdots.std(:,jj),'k--','linewidth',2);
                plot(Qref.(subjects{i}).Qdotdots.mean(:,jj)-Qref.(subjects{i}).Qdotdots.std(:,jj),'k--','linewidth',2);
                title(coordinatenames{jj},'Fontsize', 16)
            end  
            s = sgtitle('Coordinate accelerations');
            set(s,'Fontsize',20);
        end
    end
    
    if plotKinetics
        figure()
        for jj = 1:size(IDref.(subjects{i}).mean,2)
            subplot(5,4,jj)
            for c = 1:size(IDinterp.(subjects{i}),3)
                plot(IDinterp.(subjects{i})(:,jj,c),'k','linewidth',0.5); hold on
            end
            plot(IDref.(subjects{i}).mean(:,jj),'k','linewidth',2); hold on
            plot(IDref.(subjects{i}).mean(:,jj)+IDref.(subjects{i}).std(:,jj),'k--','linewidth',2);
            plot(IDref.(subjects{i}).mean(:,jj)-IDref.(subjects{i}).std(:,jj),'k--','linewidth',2);
        end 
        s = sgtitle('Joint kinetics');
        set(s,'Fontsize',20);
    end
%     
%     EMGnames = {'Ham lat','Tib ant','Per long','Gas lat','Ham med','Sol',...
%         'Vast lat','Vast med','Glu med','Rect fem','Per brev','Gas med','Add long','TFL'};
%     if plotEMG
%         figure()
%         for jj = 1:size(EMGref.(subjects{i}).mean,2)
%             subplot(4,4,jj)
%             for c = 1:size(EMGinterp.(subjects{i}),3)
%                 plot(EMGinterp.(subjects{i})(:,jj,c),'k','linewidth',0.5); hold on
%             end
%             plot(EMGref.(subjects{i}).mean(:,jj),'r','linewidth',2); hold on
%             plot(EMGref.(subjects{i}).mean(:,jj)+EMGref.(subjects{i}).std(:,jj),'b--','linewidth',2);
%             plot(EMGref.(subjects{i}).mean(:,jj)-EMGref.(subjects{i}).std(:,jj),'b--','linewidth',2);
%             title(EMGnames{jj},'Fontsize',14);
%             ylim([0 inf])
%         end 
%         s = sgtitle('EMG');
%         set(s,'Fontsize',20);
%     end   
%     
%     % Get average time
%     time_ICalls.(subjects{i}).all(1,:) = (time_ICall.(subjects{i})(:,2) - time_ICall.(subjects{i})(:,1))';
%     time_ICalls.(subjects{i}).mean = mean(time_ICalls.(subjects{i}).all);
%     time_ICalls.(subjects{i}).std = std(time_ICalls.(subjects{i}).all);
%     
%     step = time_ICalls.(subjects{i}).mean/(N-1);
%     time.(subjects{i}).mean = (0:step:time_ICalls.(subjects{i}).mean)';
%     
%     % Get average distance travelled (pelvis horizontal displacement)
%     dist_travelled.(subjects{i}).all(1,:) = Qs_spline.(subjects{i})(end,4,:);
%     dist_travelled.(subjects{i}).mean = mean(dist_travelled.(subjects{i}).all,2);
%     dist_travelled.(subjects{i}).std = std(dist_travelled.(subjects{i}).all,[],2);
%     
%     % Get average speed
%     speed.(subjects{i}).all(1,:) = dist_travelled.(subjects{i}).all./time_ICalls.(subjects{i}).all;
%     speed.(subjects{i}).mean = mean(speed.(subjects{i}).all,2);
%     speed.(subjects{i}).std = std(speed.(subjects{i}).all,[],2); 
%     
%     if createMotion
%         % Create fake motion and GRF files from the average data
%         pathOpenSim = [pathRepo,'\OpenSim'];
%         addpath(genpath(pathOpenSim));
%         JointAngle.labels = {'time'};
%         JointAngle.labels = [JointAngle.labels,joints];
%         JointAngle.data = [time.(subjects{i}).mean,Qref.(subjects{i}).Qs.mean];
%         JointAngle.data(:,[2:4,8:end]) = JointAngle.data(:,[2:4,8:end])*180/pi;
%         namescript = 'motion_ave_mtp';
%         filenameJointAngles = [pathData,'\IK\IK_',namescript,'.mot'];
%         write_motionFile(JointAngle, filenameJointAngles)        
%     end
%     
%     if createGRF
%         % Create fake motion and GRF files from the average data
%         pathOpenSim = [pathRepo,'\OpenSim'];
%         addpath(genpath(pathOpenSim));
%         
%         identifier = {'1_ground_force_v','ground_force_v'}; % Right leg: 1_ground_force_v; Left leg: ground_force_v
%         identifierp = {'1_ground_force_p','ground_force_p'}; % Right leg: 1_ground_force_p; Left leg: ground_force_p
%         identifierm = {'1_ground_torque_','ground_torque_'}; % Right leg: 1_ground_torque_; Left leg: ground_torque_
%         
%         identifier_r = {'1_ground_force_vx','1_ground_force_vy','1_ground_force_vz'};
%         identifier_l = {'ground_force_vx','ground_force_vy','ground_force_vz'};
%         JointAngle.labels = {'time'};
%         JointAngle.labels = [JointAngle.labels,identifier_r,identifier_l];
%         JointAngle.data = [time.(subjects{i}).mean,GRFref.(subjects{i}).mean];
%         identifier_pr = {'1_ground_force_px','1_ground_force_py','1_ground_force_pz'};
%         identifier_pl = {'ground_force_px','ground_force_py','ground_force_pz'};
%         JointAngle.labels = [JointAngle.labels,identifier_pr,identifier_pl];
%         JointAngle.data = [JointAngle.data,zeros(N,6)];
%         identifier_tr = {'1_ground_torque_x','1_ground_torque_y','1_ground_torque_z'};
%         identifier_tl = {'ground_torque_x','ground_torque_y','ground_torque_z'};
%         JointAngle.labels = [JointAngle.labels,identifier_tr,identifier_tl];
%         JointAngle.data = [JointAngle.data,zeros(N,6)];
%         namescript = 'motion_ave_mtp';
%         filenameJointAngles = [pathData,'\GRF\GRF_',namescript,'.mot'];
%         write_motionFile(JointAngle, filenameJointAngles)            
%     end
end
