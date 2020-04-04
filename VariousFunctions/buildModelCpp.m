clear all
close all
clc

pathmain = pwd;
[pathRepo,~,~] = fileparts(pathmain);
pathVariousFunctions = [pathRepo,'\VariousFunctions'];
addpath(genpath(pathVariousFunctions));

model_dir='EF\EF_MRI_tracking_extROM.osim';
XMLset = xml_read(model_dir);

fid=fopen('EF_MRI_tracking_extROM_BuildModelPara.txt','wt');%
strBreak='//';

%%
%01.Body
nb = numel(XMLset.Model.BodySet.objects.Body); 
for k=2:nb %without ground Ground ??
    BodyName=XMLset.Model.BodySet.objects.Body(k).ATTRIBUTE.name;
    strBodyPointer=['static OpenSim::Body* ' BodyName ';'];
    fprintf(fid,'%s\n',strBodyPointer);
end

%02. Joint
for k=2:nb 
    JointType=fieldnames(XMLset.Model.BodySet.objects.Body(k).Joint);
    eval(['Jointk', ['=XMLset.Model.BodySet.objects.Body(' num2str(k) ').Joint.' char(JointType) '']]);  
    JointName=Jointk.ATTRIBUTE.name;
     %special cases for the spine model 
        if ~strcmp(JointType, 'CustomJoint')
            JointName=[JointName '_weld']; 
        end

    strJointPointer=['static OpenSim::' char(JointType) '* ' JointName ';'];
    fprintf(fid,'%s\n',strJointPointer);
end

%%
%01.Body
for k=2:nb %without ground Ground ??
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

    strBodyPointer=['OpenSim::Body* ' BodyName ';'];
    strAddBody=['model->addBody(' num2str(BodyName) ');'];
% % %     fprintf(fid,'%s\n',strBodyPointer);
    fprintf(fid,'%s\n',BodyDefi);
    fprintf(fid,'%s\n',strAddBody,strBreak );  

end
fprintf(fid,'%s\n',strBreak);    

  
%02. Joint
count=1;
for k=2:nb
    JointType=fieldnames(XMLset.Model.BodySet.objects.Body(k).Joint);
    eval(['Jointk', ['=XMLset.Model.BodySet.objects.Body(' num2str(k) ').Joint.' char(JointType) '']]);  
    JointName=Jointk.ATTRIBUTE.name;

    JointParent=Jointk.parent_body;
    JointLocInParent=Jointk.location_in_parent;
    JointOriInParent=Jointk.orientation_in_parent;
    BodyName=XMLset.Model.BodySet.objects.Body(k).ATTRIBUTE.name;
    JointLocInChild=Jointk.location;
    JointOriInChild=Jointk.orientation;

    strJointPointer=['OpenSim::' char(JointType) '* ' JointName ';'];
% % %     fprintf(fid,'%s\n',strJointPointer);

        if strcmp(JointType, 'CustomJoint')
            %03 spatial transform
            JointSpatialTrans=['st_' BodyName '_' JointName];
            str0=['SpatialTransform ' JointSpatialTrans ';'];
            fprintf(fid,'%s\n',str0);

            nCoor=numel(Jointk.SpatialTransform.TransformAxis);
            for Coorj=1:nCoor   
                CoorNamej=Jointk.SpatialTransform.TransformAxis(Coorj).coordinates;
                CoorAxisj=Jointk.SpatialTransform.TransformAxis(Coorj).axis;
                str1=[JointSpatialTrans '[' num2str(Coorj-1) '].setCoordinateNames(OpenSim::Array<std::string>("' ...
                      CoorNamej '", 1, 1));'];
                str2=[JointSpatialTrans '[' num2str(Coorj-1) '].setFunction(new LinearFunction());'];
                str3=[JointSpatialTrans '[' num2str(Coorj-1) '].setAxis(Vec3(' ...
                num2str(CoorAxisj(1)) ',' num2str(CoorAxisj(2)) ','  num2str(CoorAxisj(3)) '));'];
                fprintf(fid,'%s\n',str1, str2, str3);
            end

            %04. Joint definition
            if strcmp(JointParent, 'ground')
                JointDefi=[JointName '= new ' char(JointType) '("' JointName '", model->getGround(), Vec3('...
                          num2str(JointLocInParent(1)) ',' num2str(JointLocInParent(2)) ','  num2str(JointLocInParent(3))  '), Vec3(' ...
                          num2str(JointOriInParent(1)) ',' num2str(JointOriInParent(2)) ','  num2str(JointOriInParent(3))  '), *' ...
                          num2str(BodyName) ', Vec3('...
                          num2str(JointLocInChild(1)) ',' num2str(JointLocInChild(2)) ','  num2str(JointLocInChild(3))  '), Vec3(' ...
                          num2str(JointOriInChild(1)) ',' num2str(JointOriInChild(2)) ','  num2str(JointOriInChild(3))  '),' ...
                          JointSpatialTrans ');'];
            else
                JointDefi=[JointName '= new ' char(JointType) '("' JointName '", *' ...
                          num2str(JointParent) ', Vec3('...
                          num2str(JointLocInParent(1)) ',' num2str(JointLocInParent(2)) ','  num2str(JointLocInParent(3))  '), Vec3(' ...
                          num2str(JointOriInParent(1)) ',' num2str(JointOriInParent(2)) ','  num2str(JointOriInParent(3))  '), *' ...
                          num2str(BodyName) ', Vec3('...
                          num2str(JointLocInChild(1)) ',' num2str(JointLocInChild(2)) ','  num2str(JointLocInChild(3))  '), Vec3(' ...
                          num2str(JointOriInChild(1)) ',' num2str(JointOriInChild(2)) ','  num2str(JointOriInChild(3))  '),' ...
                          JointSpatialTrans ');'];
            end
        else        
            if strcmp(JointParent, 'ground')
            JointDefi=[JointName '= new ' char(JointType) '("' JointName '", model->getGround(), Vec3('...
                      num2str(JointLocInParent(1)) ',' num2str(JointLocInParent(2)) ','  num2str(JointLocInParent(3))  '), Vec3(' ...
                  num2str(JointLocInParent(1)) ',' num2str(JointLocInParent(2)) ','  num2str(JointLocInParent(3))  '), Vec3(' ...
                  num2str(JointOriInParent(1)) ',' num2str(JointOriInParent(2)) ','  num2str(JointOriInParent(3))  '), *' ...
                  num2str(BodyName) ', Vec3('...
                  num2str(JointLocInChild(1)) ',' num2str(JointLocInChild(2)) ','  num2str(JointLocInChild(3))  '), Vec3(' ...
                  num2str(JointOriInChild(1)) ',' num2str(JointOriInChild(2)) ','  num2str(JointOriInChild(3))  '));'];    

            else
            JointDefi=[JointName '= new ' char(JointType) '("' JointName '", *' ...
                      num2str(JointParent) ', Vec3('...
                      num2str(JointLocInParent(1)) ',' num2str(JointLocInParent(2)) ','  num2str(JointLocInParent(3))  '), Vec3(' ...
                      num2str(JointOriInParent(1)) ',' num2str(JointOriInParent(2)) ','  num2str(JointOriInParent(3))  '), *' ...
                      num2str(BodyName) ', Vec3('...
                      num2str(JointLocInChild(1)) ',' num2str(JointLocInChild(2)) ','  num2str(JointLocInChild(3))  '), Vec3(' ...
                      num2str(JointOriInChild(1)) ',' num2str(JointOriInChild(2)) ','  num2str(JointOriInChild(3))  '));'];    
            end
        end

        fprintf(fid,'%s\n',JointDefi);

        strAddJoint=['model->addJoint(' num2str(JointName) ');'];    
        fprintf(fid,'%s\n',strAddJoint,strBreak );  
        count=count+1;
end
    

fclose(fid);