% Currently, our framework does not allow loading an osim file when creating an
% external function in C++. The user is therefore left with the tedious
% task of building the model from scratch, which is annoying and prone to
% error. This file aims to facilitate this process by writing a txt file with
% part of the C++ code required to build a model form scratch.
%
% Note that this script has not been tested extensively so you should always
% double check the output to make sure everything makes senses. It is
% indeed very likely that not all cases are accounted for. For instance, at
% this stage coordinates that are locked will not be locked in the C++ file.
%
% Note that this code assumes the .osim file to follow the OpenSim 3.3 syntax.
% Things have changed from OpenSim 4.0 but this script has not been
% adjusted yet. But importantly the resulting cpp code matches >=4.0
% conventions.
%
% Please report any bugs so that we can make progress.
%
% Author: Echo (Wei) Wang and Antoine Falisse
% Date: 06/04/2020

clear all
close all
clc

%% User settings
pathmain = pwd;
[pathRepo,~,~] = fileparts(pathmain);
% Path to your osim file
pathModel = [pathRepo,'\OpenSimModel\subject1\subject1_v2.osim'];
% Path to where you want to save the output txt file
pathOutput = [pathRepo,'\OpenSimModel\subject1\subject1_v2.txt'];
% Load the model
pathVariousFunctions = [pathRepo,'\VariousFunctions'];
addpath(genpath(pathVariousFunctions));
XMLset = xml_read(pathModel);
fid=fopen(pathOutput,'w+');
strBreak='//';
% When building the model in C++, you should order the joints according to how
% they are ordered in your main script. For instance, in PredSim_all_v2, we have 
% the following order: ground_pelvis (tilt, list, rot, tx, ty, tz), hip_l
% (flex, add, rot), hip_r (flex, add, rot), knee_l, knee_r, ankle_l, ankle_r,
% subtalar_l, subtalar_r, back, shoulder_l (flex, add, rot), shoulder_r (flex,
% add, rot), elbow_l, elbow_r, radioulnar_l, radioulnar_r. See section "Indices
% external function".
%
% This list includes locked joints (radioulnars) but the joints are not
% actually locked in the C++ file (external function). A workaround is to
% pass constant values as input to the external function so that the locked
% coordinates do not change.
%
% WeldJoints are not included in the list above, since the coordinate
% values and speeds do not change. Yet you should include those
% joints in your model, as done in the osim file.
%
% JointNames contains the names of the joints in the order found in the
% osim file. Please adjust this order by filling out JointOrder as you
% which. For instance, in PredSim_v2, we want ground_pelvis=1, hip_l=2,
% etc.
nb = numel(XMLset.Model.BodySet.objects.Body);
JointNames = cell(1,nb-1);
for k=2:nb % from 2 cause 1 is ground
    JointType=fieldnames(XMLset.Model.BodySet.objects.Body(k).Joint);
    Jointk=XMLset.Model.BodySet.objects.Body(k).Joint.(JointType{1,1}); 
    JointName=Jointk.ATTRIBUTE.name;
    JointNames{k-1} = JointName;
end
JointOrder = {'ground_pelvis','hip_l','hip_r','knee_l','knee_r','ankle_l',...
    'ankle_r','subtalar_l','subtalar_r','mtp_l','mtp_r','back','shoulder_l',...
    'shoulder_r','elbow_l','elbow_r','radioulnar_l','radioulnar_r',...
    'radius_hand_l','radius_hand_r'};
for i = 1:length(JointOrder)
    if isempty(find(strcmp(JointOrder{i},JointNames),1))
        error([JointOrder{i} ' is not the model']);
    end
end

%% Create model components
fprintf(fid,'%s\n', '// OpenSim: create components');
% Model
fprintf(fid,'%s\n', '/// Model');
strModelPointer='OpenSim::Model* model;';
fprintf(fid,'%s\n',strModelPointer);
% Bodies
fprintf(fid,'%s\n', '/// Bodies');
nb = numel(XMLset.Model.BodySet.objects.Body); 
for k=2:nb % from 2 cause 1 is ground
    BodyName=XMLset.Model.BodySet.objects.Body(k).ATTRIBUTE.name;
    strBodyPointer=['OpenSim::Body* ' BodyName ';'];
    fprintf(fid,'%s\n',strBodyPointer);
end
% Joints
fprintf(fid,'%s\n', '/// Joints');
for k=2:nb % from 2 cause 1 is ground
    JointType=fieldnames(XMLset.Model.BodySet.objects.Body(k).Joint);
    Jointk=XMLset.Model.BodySet.objects.Body(k).Joint.(JointType{1,1});  
    JointName=Jointk.ATTRIBUTE.name;
    strJointPointer=['OpenSim::' char(JointType) '* ' JointName ';'];
    fprintf(fid,'%s\n',strJointPointer);
end

%% Initialize model components
% Model
fprintf(fid,'%s\n', '/// Model');
strModelPointer='model = new OpenSim::Model();';
fprintf(fid,'%s\n',strModelPointer);
% Bodies
fprintf(fid,'%s\n', '/// Bodies');
for k=2:nb % from 2 cause 1 is ground
    BodyName=XMLset.Model.BodySet.objects.Body(k).ATTRIBUTE.name;
    BodyMass=XMLset.Model.BodySet.objects.Body(k).mass;
    BodyMCoM=XMLset.Model.BodySet.objects.Body(k).mass_center; %need ','?
    BodyInertia=[XMLset.Model.BodySet.objects.Body(k).inertia_xx,...
                XMLset.Model.BodySet.objects.Body(k).inertia_yy,...
                XMLset.Model.BodySet.objects.Body(k).inertia_zz,...
                XMLset.Model.BodySet.objects.Body(k).inertia_xy,...
                XMLset.Model.BodySet.objects.Body(k).inertia_xz,...
                XMLset.Model.BodySet.objects.Body(k).inertia_yz];
    BodyDefi=[BodyName '= new OpenSim::Body("' BodyName '",' ...
              num2str(BodyMass) ', Vec3('...
              num2str(BodyMCoM(1)) ',' num2str(BodyMCoM(2)) ','  num2str(BodyMCoM(3))  '), Inertia(' ...
              num2str(BodyInertia(1)) ',' num2str(BodyInertia(2)) ',' num2str(BodyInertia(3)) ',' ...
              num2str(BodyInertia(4)) ',' num2str(BodyInertia(5)) ',' num2str(BodyInertia(6)) '));'];
    strAddBody=['model->addBody(' num2str(BodyName) ');'];
    fprintf(fid,'%s\n',BodyDefi);
    fprintf(fid,'%s\n',strAddBody); 
end
% Joints
fprintf(fid,'%s\n', '/// Joints');
for k=2:nb % from 2 cause 1 is ground
    JointType=fieldnames(XMLset.Model.BodySet.objects.Body(k).Joint);
    Jointk=XMLset.Model.BodySet.objects.Body(k).Joint.(JointType{1,1}); 
    JointName=Jointk.ATTRIBUTE.name;
    JointParent=Jointk.parent_body;
    JointLocInParent=Jointk.location_in_parent;
    JointOriInParent=Jointk.orientation_in_parent;
    BodyName=XMLset.Model.BodySet.objects.Body(k).ATTRIBUTE.name;
    JointLocInChild=Jointk.location;
    JointOriInChild=Jointk.orientation;
    fprintf(fid,'%s\n', ['//', JointName]);
    if strcmp(JointType, 'CustomJoint')
        % Spatial transform
        JointSpatialTrans=['st_' JointName];
        str0=['SpatialTransform ' JointSpatialTrans ';'];
        fprintf(fid,'%s\n',str0);
        nCoor=numel(Jointk.SpatialTransform.TransformAxis);
        for Coorj=1:nCoor  
            CoorNamej=Jointk.SpatialTransform.TransformAxis(Coorj).coordinates;
            CoorAxisj=Jointk.SpatialTransform.TransformAxis(Coorj).axis;               
            CoorFunctionj=fieldnames(Jointk.SpatialTransform.TransformAxis(Coorj).xFunction);
            if strcmp(CoorFunctionj{1,1},'LinearFunction') 
                if all(Jointk.SpatialTransform.TransformAxis(Coorj).xFunction.(CoorFunctionj{1,1}).coefficients == [1, 0])              
                    str1=[JointSpatialTrans '[' num2str(Coorj-1) '].setCoordinateNames(OpenSim::Array<std::string>("' ...
                          CoorNamej '", 1, 1));'];
                    str2=[JointSpatialTrans '[' num2str(Coorj-1) '].setFunction(new ' CoorFunctionj{1,1} '());'];
                    str3=[JointSpatialTrans '[' num2str(Coorj-1) '].setAxis(Vec3(' ...
                    num2str(CoorAxisj(1)) ',' num2str(CoorAxisj(2)) ','  num2str(CoorAxisj(3)) '));'];
                    fprintf(fid,'%s\n',str1, str2, str3);
                else
                    error("Linear functions with coefficients different than [1, 0] are not supported in the code")   
                end
            elseif strcmp(CoorFunctionj{1,1},'Constant') 
                CoorFunctionValuej=Jointk.SpatialTransform.TransformAxis(Coorj).xFunction.(CoorFunctionj{1,1}).value;
                str2=[JointSpatialTrans '[' num2str(Coorj-1) '].setFunction(new ' CoorFunctionj{1,1} '(' num2str(CoorFunctionValuej) '));'];
                str3=[JointSpatialTrans '[' num2str(Coorj-1) '].setAxis(Vec3(' ...
                num2str(CoorAxisj(1)) ',' num2str(CoorAxisj(2)) ','  num2str(CoorAxisj(3)) '));'];
                fprintf(fid,'%s\n',str2, str3);
            elseif strcmp(CoorFunctionj{1,1},'MultiplierFunction') 
                CoorFunctionbj=fieldnames(Jointk.SpatialTransform.TransformAxis(Coorj).xFunction.MultiplierFunction.xFunction);
                if strcmp(CoorFunctionbj{1,1},'Constant')
                    CoorFunctionValuej=Jointk.SpatialTransform.TransformAxis(Coorj).xFunction.MultiplierFunction.xFunction.(CoorFunctionbj{1,1}).value;
                    scaleValuej = Jointk.SpatialTransform.TransformAxis(Coorj).xFunction.MultiplierFunction.scale;
                    str2=[JointSpatialTrans '[' num2str(Coorj-1) '].setFunction(new ' CoorFunctionj{1,1} '(new ' CoorFunctionbj{1,1}  '(' num2str(CoorFunctionValuej) '), ' num2str(scaleValuej) '));'];
                    str3=[JointSpatialTrans '[' num2str(Coorj-1) '].setAxis(Vec3(' ...
                    num2str(CoorAxisj(1)) ',' num2str(CoorAxisj(2)) ','  num2str(CoorAxisj(3)) '));'];
                    fprintf(fid,'%s\n', str2, str3);
                else
                    error("Multiplier functions with functions different than constant are not supported in the code")   
                end
            else
                error("Function type used in transform not supported")                   
            end 
        end
        % Joint definition
        if strcmp(JointParent, 'ground')
            JointDefi=[JointName ' = new ' char(JointType) '("' JointName '", model->getGround(), Vec3('...
                      num2str(JointLocInParent(1)) ',' num2str(JointLocInParent(2)) ','  num2str(JointLocInParent(3))  '), Vec3(' ...
                      num2str(JointOriInParent(1)) ',' num2str(JointOriInParent(2)) ','  num2str(JointOriInParent(3))  '), *' ...
                      num2str(BodyName) ', Vec3('...
                      num2str(JointLocInChild(1)) ',' num2str(JointLocInChild(2)) ','  num2str(JointLocInChild(3))  '), Vec3(' ...
                      num2str(JointOriInChild(1)) ',' num2str(JointOriInChild(2)) ','  num2str(JointOriInChild(3))  '),' ...
                      JointSpatialTrans ');'];
        else
            JointDefi=[JointName ' = new ' char(JointType) '("' JointName '", *', num2str(JointParent) ', Vec3('...
                      num2str(JointLocInParent(1)) ',' num2str(JointLocInParent(2)) ','  num2str(JointLocInParent(3))  '), Vec3(' ...
                      num2str(JointOriInParent(1)) ',' num2str(JointOriInParent(2)) ','  num2str(JointOriInParent(3))  '), *' ...
                      num2str(BodyName) ', Vec3('...
                      num2str(JointLocInChild(1)) ',' num2str(JointLocInChild(2)) ','  num2str(JointLocInChild(3))  '), Vec3(' ...
                      num2str(JointOriInChild(1)) ',' num2str(JointOriInChild(2)) ','  num2str(JointOriInChild(3))  '),' ...
                      JointSpatialTrans ');'];
        end
    else      
        if strcmp(JointParent, 'ground')
            JointDefi=[JointName ' = new ' char(JointType) '("' JointName '", model->getGround(), Vec3('...
                      num2str(JointLocInParent(1)) ',' num2str(JointLocInParent(2)) ','  num2str(JointLocInParent(3))  '), Vec3(' ...
                  num2str(JointLocInParent(1)) ',' num2str(JointLocInParent(2)) ','  num2str(JointLocInParent(3))  '), Vec3(' ...
                  num2str(JointOriInParent(1)) ',' num2str(JointOriInParent(2)) ','  num2str(JointOriInParent(3))  '), *' ...
                  num2str(BodyName) ', Vec3('...
                  num2str(JointLocInChild(1)) ',' num2str(JointLocInChild(2)) ','  num2str(JointLocInChild(3))  '), Vec3(' ...
                  num2str(JointOriInChild(1)) ',' num2str(JointOriInChild(2)) ','  num2str(JointOriInChild(3))  '));'];    

        else
            JointDefi=[JointName ' = new ' char(JointType) '("' JointName '", *' ...
                      num2str(JointParent) ', Vec3('...
                      num2str(JointLocInParent(1)) ',' num2str(JointLocInParent(2)) ','  num2str(JointLocInParent(3))  '), Vec3(' ...
                      num2str(JointOriInParent(1)) ',' num2str(JointOriInParent(2)) ','  num2str(JointOriInParent(3))  '), *' ...
                      num2str(BodyName) ', Vec3('...
                      num2str(JointLocInChild(1)) ',' num2str(JointLocInChild(2)) ','  num2str(JointLocInChild(3))  '), Vec3(' ...
                      num2str(JointOriInChild(1)) ',' num2str(JointOriInChild(2)) ','  num2str(JointOriInChild(3))  '));'];    
        end
    end
    fprintf(fid,'%s\n',JointDefi); 
end    
fprintf(fid,'%s\n', '/// Add joints');
for i =1:length(JointOrder)
    strAddJoint=['model->addJoint(' num2str(JointOrder{i}) ');'];
    fprintf(fid,'%s\n',strAddJoint );
end

fclose(fid);
