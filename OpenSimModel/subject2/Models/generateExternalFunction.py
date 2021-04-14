import os
import sys
sys.path.append("..") # utilities in child directory
import opensim
import numpy as np

# %% Paths.
scriptDir = os.getcwd()
modelDir = os.path.dirname(scriptDir) 
OpenSimModelDir = os.path.dirname(modelDir)

# %% User settings.

modelName = "subject2_withMTP_weldRadius_scaled_FK_contactsAsForces_optimized"
pathModel = os.path.join(scriptDir, modelName + ".osim")
pathModelFolder = scriptDir

jointsOrder = ['ground_pelvis', 'hip_l', 'hip_r', 'knee_l', 'knee_r', 'ankle_l',
              'ankle_r', 'subtalar_l', 'subtalar_r', 'mtp_l', 'mtp_r', 'back',
              'acromial_l', 'acromial_r', 'elbow_l', 'elbow_r', 'radioulnar_l',
              'radioulnar_r', 'radius_hand_l', 'radius_hand_r']

coordinatesOrder = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation',
                    'pelvis_tx', 'pelvis_ty', 'pelvis_tz', 'hip_flexion_l',
                    'hip_adduction_l', 'hip_rotation_l', 'hip_flexion_r',
                    'hip_adduction_r', 'hip_rotation_r', 'knee_angle_l',
                    'knee_angle_r','ankle_angle_l', 'ankle_angle_r',
                    'subtalar_angle_l', 'subtalar_angle_r',
                    'lumbar_extension', 'lumbar_bending',
                    'lumbar_rotation', 'arm_flex_l', 'arm_add_l', 'arm_rot_l', 
                    'arm_flex_r', 'arm_add_r', 'arm_rot_r', 'elbow_flex_l', 
                    'elbow_flex_r', 'pro_sup_l', 'pro_sup_r']

''' 
    Set True to build external function.
    TODO
'''
buildExternalFunction = False

''' 
    Set True to verify that external function returns the same torques as the
    ID tool together with the .osim file. This check ensures that the model
    has been built (programmatically) correctly.
'''
verifyID = False

'''
    Set True to generate post_processing external function. In such case, 
    exportMarkerPositions is set to True by default.
'''
exportMarkerPositions = False
response_markers = []

generate_pp = False

# %% Various operations based on user settings.

# outputModelFileName = (OpenSimModel + "_scaled_" +  "Case_" + prefix_model + 
#                        "_" + hyperparameters + "_" + model_type + suffix_model)
# pathOutputFile = (pathModel[:-4] + ".cpp")
# pathOutputFiles = os.path.join(pathModelFolder, outputModelFileName)
pathOutputExternalFunctionFolder = os.path.join(pathModelFolder,
                                                "ExternalFunction")
if not os.path.exists(pathOutputExternalFunctionFolder):
    os.makedirs(pathOutputExternalFunctionFolder)
pathOutputFile = os.path.join(pathOutputExternalFunctionFolder,
                              modelName + ".cpp")

# %% Generate external Function (.cpp file)
model = opensim.Model(pathModel)
model.initSystem()
bodySet = model.getBodySet()
jointSet = model.get_JointSet()
geometrySet = model.get_ContactGeometrySet()
forceSet = model.get_ForceSet()
coordinateSet = model.getCoordinateSet()
nCoordinates = coordinateSet.getSize()

nContacts = 0
for i in range(forceSet.getSize()):        
    c_force_elt = forceSet.get(i)        
    if c_force_elt.getConcreteClassName() == "SmoothSphereHalfSpaceForce":  
        nContacts += 1

with open(pathOutputFile, "w") as f:
    
    f.write('#include <OpenSim/Simulation/Model/Model.h>\n')
    f.write('#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>\n')
    f.write('#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>\n')
    f.write('#include <OpenSim/Simulation/SimbodyEngine/Joint.h>\n')
    f.write('#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>\n')
    f.write('#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>\n')
    f.write('#include <OpenSim/Common/LinearFunction.h>\n')
    f.write('#include <OpenSim/Common/Constant.h>\n')
    f.write('#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>\n')
    f.write('#include "SimTKcommon/internal/recorder.h"\n\n')
    
    f.write('#include <iostream>\n')
    f.write('#include <iterator>\n')
    f.write('#include <random>\n')
    f.write('#include <cassert>\n')
    f.write('#include <algorithm>\n')
    f.write('#include <vector>\n')
    f.write('#include <fstream>\n\n')
    
    f.write('using namespace SimTK;\n')
    f.write('using namespace OpenSim;\n\n')

    f.write('constexpr int n_in = 2; \n')
    f.write('constexpr int n_out = 1; \n')
    
    f.write('constexpr int nCoordinates = %i; \n' % nCoordinates)
    f.write('constexpr int NX = nCoordinates*2; \n')
    f.write('constexpr int NU = nCoordinates; \n')
    
    
    if exportMarkerPositions:
        nMarkers = len(response_markers)
        if generate_pp:
            f.write('constexpr int NR = nCoordinates + 3*%i + 3*4 + 3*2*%i; \n\n' % (nMarkers, nContacts))
        else:
            f.write('constexpr int NR = nCoordinates + 3*%i; \n\n' % (nMarkers))
        
    else:
        f.write('constexpr int NR = nCoordinates; \n\n')
    
    f.write('template<typename T> \n')
    f.write('T value(const Recorder& e) { return e; }; \n')
    f.write('template<> \n')
    f.write('double value(const Recorder& e) { return e.getValue(); }; \n\n')
    
    f.write('SimTK::Array_<int> getIndicesOSInSimbody(const Model& model) { \n')
    f.write('\tauto s = model.getWorkingState(); \n')
    f.write('\tconst auto svNames = model.getStateVariableNames(); \n')
    f.write('\tSimTK::Array_<int> idxOSInSimbody(s.getNQ()); \n')
    f.write('\ts.updQ() = 0; \n')
    f.write('\tfor (int iy = 0; iy < s.getNQ(); ++iy) { \n')
    f.write('\t\ts.updQ()[iy] = SimTK::NaN; \n')
    f.write('\t\tconst auto svValues = model.getStateVariableValues(s); \n')
    f.write('\t\tfor (int isv = 0; isv < svNames.size(); ++isv) { \n')
    f.write('\t\t\tif (SimTK::isNaN(svValues[isv])) { \n')
    f.write('\t\t\t\ts.updQ()[iy] = 0; \n')
    f.write('\t\t\t\tidxOSInSimbody[iy] = isv/2; \n')
    f.write('\t\t\t\tbreak; \n')
    f.write('\t\t\t} \n')
    f.write('\t\t} \n')
    f.write('\t} \n')
    f.write('\treturn idxOSInSimbody; \n')
    f.write('} \n\n')
    
    f.write('SimTK::Array_<int> getIndicesSimbodyInOS(const Model& model) { \n')
    f.write('\tauto idxOSInSimbody = getIndicesOSInSimbody(model); \n')
    f.write('\tauto s = model.getWorkingState(); \n')
    f.write('\tSimTK::Array_<int> idxSimbodyInOS(s.getNQ()); \n')
    f.write('\tfor (int iy = 0; iy < s.getNQ(); ++iy) { \n')
    f.write('\t\tfor (int iyy = 0; iyy < s.getNQ(); ++iyy) { \n')
    f.write('\t\t\tif (idxOSInSimbody[iyy] == iy) { \n')
    f.write('\t\t\t\tidxSimbodyInOS[iy] = iyy; \n')
    f.write('\t\t\t\tbreak; \n')
    f.write('\t\t\t} \n')
    f.write('\t\t} \n')
    f.write('\t} \n')	
    f.write('\treturn idxSimbodyInOS; \n')
    f.write('} \n\n')
    
    f.write('template<typename T>\n')
    f.write('int F_generic(const T** arg, T** res) {\n')
    
    f.write('\tOpenSim::Model* model;\n')
    f.write('\tmodel = new OpenSim::Model();;\n\n')
    
    # Bodies
    f.write('\t// Definition of bodies\n')
    for i in range(bodySet.getSize()):        
        c_body = bodySet.get(i)
        c_body_name = c_body.getName()
        c_body_mass = c_body.get_mass()
        c_body_mass_center = c_body.get_mass_center().to_numpy()
        c_body_inertia = c_body.get_inertia()
        c_body_inertia_vec3 = np.array([c_body_inertia.get(0), c_body_inertia.get(1), c_body_inertia.get(2)])        
        f.write('\tOpenSim::Body* %s;\n' % c_body_name)
        f.write('\t%s = new OpenSim::Body(\"%s\", %.20f, Vec3(%.20f, %.20f, %.20f), Inertia(%.20f, %.20f, %.20f, 0., 0., 0.));\n' % (c_body_name, c_body_name, c_body_mass, c_body_mass_center[0], c_body_mass_center[1], c_body_mass_center[2], c_body_inertia_vec3[0], c_body_inertia_vec3[1], c_body_inertia_vec3[2]))
        f.write('\tmodel->addBody(%s);\n' % (c_body_name))
        f.write('\n')   
    
    # Joints
    f.write('\t// Definition of joints\n')
    for i in range(jointSet.getSize()): 
        c_joint = jointSet.get(i)        
        nJointCoordinates = int(c_joint.getNumStateVariables() / 2)
        
        parent_frame = c_joint.get_frames(0)
        parent_frame_name = parent_frame.getParentFrame().getName()
        parent_frame_trans = parent_frame.get_translation().to_numpy()
        parent_frame_or = parent_frame.get_orientation().to_numpy()
        
        child_frame = c_joint.get_frames(1)
        child_frame_name = child_frame.getParentFrame().getName()
        child_frame_trans = child_frame.get_translation().to_numpy()
        child_frame_or = child_frame.get_orientation().to_numpy()
        
        # Custom joints
        if c_joint.getConcreteClassName() == "CustomJoint":
            
            f.write('\tSpatialTransform st_%s;\n' % c_joint.getName())
            
            cObj = opensim.CustomJoint.safeDownCast(c_joint)    
            spatialtransform = cObj.get_SpatialTransform()
            
            rot1 = spatialtransform.get_rotation1()
            rot1_axis = rot1.get_axis().to_numpy()
            rot2 = spatialtransform.get_rotation2()
            rot2_axis = rot2.get_axis().to_numpy()
            rot3 = spatialtransform.get_rotation3()
            rot3_axis = rot3.get_axis().to_numpy()
            tr1 = spatialtransform.get_translation1()
            tr1_axis = tr1.get_axis().to_numpy()
            tr2 = spatialtransform.get_translation2()
            tr2_axis = tr2.get_axis().to_numpy()
            tr3 = spatialtransform.get_translation3()
            tr3_axis = tr3.get_axis().to_numpy()
            
            c_axes = np.zeros((6,3))
            c_axes[0,:] = rot1_axis
            c_axes[1,:] = rot2_axis
            c_axes[2,:] = rot3_axis
            c_axes[3,:] = tr1_axis
            c_axes[4,:] = tr2_axis
            c_axes[5,:] = tr3_axis
            
            for coord in range(6):                            
                if coord < nJointCoordinates:
                    c_coord = c_joint.get_coordinates(coord)
                    c_coord_name = c_coord.getName()
                    f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                    f.write('\tst_%s[%i].setFunction(new LinearFunction());\n' % (c_joint.getName(), coord))
                else:
                    f.write('\tst_%s[%i].setFunction(new Constant(0));\n' % (c_joint.getName(), coord))                    
                f.write('\tst_%s[%i].setAxis(Vec3(%.20f, %.20f, %.20f));\n' % (c_joint.getName(), coord, c_axes[coord, 0], c_axes[coord, 1], c_axes[coord, 2]))              
            f.write('\tOpenSim::%s* %s;\n' % (c_joint.getConcreteClassName(), c_joint.getName()))            
        
            if parent_frame_name == "ground":
                f.write('\t%s = new OpenSim::%s(\"%s\", model->getGround(), Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), st_%s);\n' % (c_joint.getName(), c_joint.getConcreteClassName(), c_joint.getName(), parent_frame_trans[0], parent_frame_trans[1], parent_frame_trans[2], parent_frame_or[0], parent_frame_or[1], parent_frame_or[2], child_frame_name, child_frame_trans[0], child_frame_trans[1], child_frame_trans[2], child_frame_or[0], child_frame_or[1], child_frame_or[2], c_joint.getName()))     
            else:
                f.write('\t%s = new OpenSim::%s(\"%s\", *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), st_%s);\n' % (c_joint.getName(), c_joint.getConcreteClassName(), c_joint.getName(), parent_frame_name, parent_frame_trans[0], parent_frame_trans[1], parent_frame_trans[2], parent_frame_or[0], parent_frame_or[1], parent_frame_or[2], child_frame_name, child_frame_trans[0], child_frame_trans[1], child_frame_trans[2], child_frame_or[0], child_frame_or[1], child_frame_or[2], c_joint.getName()))
            
        else:
            f.write('\tOpenSim::%s* %s;\n' % (c_joint.getConcreteClassName(), c_joint.getName()))
            if parent_frame_name == "ground":
                f.write('\t%s = new OpenSim::%s(\"%s\", model->getGround(), Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f));\n' % (c_joint.getName(), c_joint.getConcreteClassName(), c_joint.getName(), parent_frame_trans[0], parent_frame_trans[1], parent_frame_trans[2], parent_frame_or[0], parent_frame_or[1], parent_frame_or[2], child_frame_name, child_frame_trans[0], child_frame_trans[1], child_frame_trans[2], child_frame_or[0], child_frame_or[1], child_frame_or[2]))     
            else:
                f.write('\t%s = new OpenSim::%s(\"%s\", *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f));\n' % (c_joint.getName(), c_joint.getConcreteClassName(), c_joint.getName(), parent_frame_name, parent_frame_trans[0], parent_frame_trans[1], parent_frame_trans[2], parent_frame_or[0], parent_frame_or[1], parent_frame_or[2], child_frame_name, child_frame_trans[0], child_frame_trans[1], child_frame_trans[2], child_frame_or[0], child_frame_or[1], child_frame_or[2])) 
        f.write('\n')
    # Add joints to model in pre-defined order
    if jointsOrder:
        for jointOrder in jointsOrder: 
            f.write('\tmodel->addJoint(%s);\n' % (jointOrder))
            try:
                c_joint = jointSet.get(jointOrder)
            except:
                raise ValueError("Joint from jointOrder not in jointSet")                
        assert(len(jointsOrder) == jointSet.getSize()), "jointsOrder and jointSet have different sizes"
    f.write('\n')    
            
    # Contacts
    f.write('\t// Definition of contacts\n')   
    for i in range(forceSet.getSize()):        
        c_force_elt = forceSet.get(i)        
        if c_force_elt.getConcreteClassName() == "SmoothSphereHalfSpaceForce":            
            c_force_elt_obj =  opensim.SmoothSphereHalfSpaceForce.safeDownCast(c_force_elt) 	
            
            socket0Name = c_force_elt.getSocketNames()[0]
            socket0 = c_force_elt.getSocket(socket0Name)
            socket0_obj = socket0.getConnecteeAsObject()
            socket0_objName = socket0_obj.getName()            
            geo0 = geometrySet.get(socket0_objName)
            geo0_loc = geo0.get_location().to_numpy()
            geo0_or = geo0.get_orientation().to_numpy()
            geo0_frameName = geo0.getFrame().getName()
            
            socket1Name = c_force_elt.getSocketNames()[1]
            socket1 = c_force_elt.getSocket(socket1Name)
            socket1_obj = socket1.getConnecteeAsObject()
            socket1_objName = socket1_obj.getName()            
            geo1 = geometrySet.get(socket1_objName)
            geo1_loc = geo1.get_location().to_numpy()
            geo1_or = geo1.get_orientation().to_numpy()
            geo1_frameName = geo1.getFrame().getName()
            obj = opensim.ContactSphere.safeDownCast(geo1) 	
            geo1_radius = obj.getRadius()            
            
            f.write('\tOpenSim::%s* %s;\n' % (c_force_elt.getConcreteClassName(), c_force_elt.getName()))
            if geo0_frameName == "ground":
                f.write('\t%s = new %s(\"%s\", *%s, model->getGround());\n' % (c_force_elt.getName(), c_force_elt.getConcreteClassName(), c_force_elt.getName(), geo1_frameName))
            else:
                f.write('\t%s = new %s(\"%s\", *%s, *%s);\n' % (c_force_elt.getName(), c_force_elt.getConcreteClassName(), c_force_elt.getName(), geo1_frameName, geo0_frameName))
                
            f.write('\tVec3 %s_location(%.20f, %.20f, %.20f);\n' % (c_force_elt.getName(), geo1_loc[0], geo1_loc[1], geo1_loc[2]))
            f.write('\t%s->set_contact_sphere_location(%s_location);\n' % (c_force_elt.getName(), c_force_elt.getName()))
            f.write('\tdouble %s_radius = (%.20f);\n' % (c_force_elt.getName(), geo1_radius))
            f.write('\t%s->set_contact_sphere_radius(%s_radius );\n' % (c_force_elt.getName(), c_force_elt.getName()))
            f.write('\t%s->set_contact_half_space_location(Vec3(%.20f, %.20f, %.20f));\n' % (c_force_elt.getName(), geo0_loc[0], geo0_loc[1], geo0_loc[2]))
            f.write('\t%s->set_contact_half_space_orientation(Vec3(%.20f, %.20f, %.20f));\n' % (c_force_elt.getName(), geo0_or[0], geo0_or[1], geo0_or[2]))
            
            f.write('\t%s->set_stiffness(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_stiffness()))
            f.write('\t%s->set_dissipation(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_dissipation()))
            f.write('\t%s->set_static_friction(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_static_friction()))
            f.write('\t%s->set_dynamic_friction(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_dynamic_friction()))
            f.write('\t%s->set_viscous_friction(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_viscous_friction()))
            f.write('\t%s->set_transition_velocity(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_transition_velocity()))
            
            f.write('\t%s->connectSocket_sphere_frame(*%s);\n' % (c_force_elt.getName(), geo1_frameName))
            if geo0_frameName == "ground":
                f.write('\t%s->connectSocket_half_space_frame(model->getGround());\n' % (c_force_elt.getName()))                
            else:
                f.write('\t%s->connectSocket_half_space_frame(*%s);\n' % (c_force_elt.getName(), geo0_frameName))
            f.write('\tmodel->addComponent(%s);\n' % (c_force_elt.getName()))
            f.write('\n')
            
    # Markers
    if exportMarkerPositions:
        markerSet = model.get_MarkerSet()
        for marker in response_markers: 
            if "." in marker:
                marker_adj = marker.replace(".", "_")
            else:
                marker_adj = marker                
            c_marker_loc = markerSet.get(marker).get_location().to_numpy()
            c_marker_parent = markerSet.get(marker).getParentFrame().getName()            
            f.write('\tOpenSim::Station* %s;\n' % marker_adj)
            f.write('\t%s = new Station(*%s, Vec3(%.20f, %.20f, %.20f));\n' % (marker_adj, c_marker_parent, c_marker_loc[0], c_marker_loc[1], c_marker_loc[2]))
            f.write('\tmodel->addComponent(%s);\n' % (marker_adj))
        f.write('\n')
            
    f.write('\t// Initialize system.\n')
    f.write('\tSimTK::State* state;\n')
    f.write('\tstate = new State(model->initSystem());\n\n')

    f.write('\t// Read inputs.\n')
    f.write('\tstd::vector<T> x(arg[0], arg[0] + NX);\n')
    f.write('\tstd::vector<T> u(arg[1], arg[1] + NU);\n\n')
    
    f.write('\t// States and controls.\n')
    f.write('\tT ua[NU];\n')
    f.write('\tVector QsUs(NX);\n')
    f.write('\t/// States\n')
    f.write('\tfor (int i = 0; i < NX; ++i) QsUs[i] = x[i];\n') 	
    f.write('\t/// Controls\n')
    f.write('\t/// OpenSim and Simbody have different state orders.\n')
    f.write('\tauto indicesOSInSimbody = getIndicesOSInSimbody(*model);\n')
    f.write('\tfor (int i = 0; i < NU; ++i) ua[i] = u[indicesOSInSimbody[i]];\n\n')

    f.write('\t// Set state variables and realize.\n')
    f.write('\tmodel->setStateVariableValues(*state, QsUs);\n')
    f.write('\tmodel->realizeVelocity(*state);\n\n')
    
    f.write('\t// Compute residual forces.\n')
    f.write('\t/// Set appliedMobilityForces (# mobilities).\n')
    f.write('\tVector appliedMobilityForces(nCoordinates);\n')
    f.write('\tappliedMobilityForces.setToZero();\n')
    f.write('\t/// Set appliedBodyForces (# bodies + ground).\n')
    f.write('\tVector_<SpatialVec> appliedBodyForces;\n')
    f.write('\tint nbodies = model->getBodySet().getSize() + 1;\n')
    f.write('\tappliedBodyForces.resize(nbodies);\n')
    f.write('\tappliedBodyForces.setToZero();\n')
    f.write('\t/// Set gravity.\n')
    f.write('\tVec3 gravity(0);\n')
    f.write('\tgravity[1] = %.20f;\n' % model.get_gravity()[1])
    f.write('\t/// Add weights to appliedBodyForces.\n')
    f.write('\tfor (int i = 0; i < model->getBodySet().getSize(); ++i) {\n')
    f.write('\t\tmodel->getMatterSubsystem().addInStationForce(*state,\n')
    f.write('\t\tmodel->getBodySet().get(i).getMobilizedBodyIndex(),\n')
    f.write('\t\tmodel->getBodySet().get(i).getMassCenter(),\n')
    f.write('\t\tmodel->getBodySet().get(i).getMass()*gravity, appliedBodyForces);\n')
    f.write('\t}\n')    
    f.write('\t/// Add contact forces to appliedBodyForces.\n')
    
    count = 0
    for i in range(forceSet.getSize()):        
        c_force_elt = forceSet.get(i)     
        
        if c_force_elt.getConcreteClassName() == "SmoothSphereHalfSpaceForce":
            c_force_elt_name = c_force_elt.getName()    
            
            f.write('\tArray<osim_double_adouble> Force_%s = %s->getRecordValues(*state);\n' % (str(count), c_force_elt_name))
            f.write('\tSpatialVec GRF_%s;\n' % (str(count)))           
            
            f.write('\tGRF_%s[0] = Vec3(Force_%s[3], Force_%s[4], Force_%s[5]);\n' % (str(count), str(count), str(count), str(count)))
            f.write('\tGRF_%s[1] = Vec3(Force_%s[0], Force_%s[1], Force_%s[2]);\n' % (str(count), str(count), str(count), str(count)))
            
            socket1Name = c_force_elt.getSocketNames()[1]
            socket1 = c_force_elt.getSocket(socket1Name)
            socket1_obj = socket1.getConnecteeAsObject()
            socket1_objName = socket1_obj.getName()            
            geo1 = geometrySet.get(socket1_objName)
            geo1_frameName = geo1.getFrame().getName()
            
            f.write('\tint c_idx_%s = model->getBodySet().get("%s").getMobilizedBodyIndex();\n' % (str(count), geo1_frameName))            
            f.write('\tappliedBodyForces[c_idx_%s] += GRF_%s;\n' % (str(count), str(count)))
            count += 1
            f.write('\n')
            
    f.write('\t/// knownUdot.\n')
    f.write('\tVector knownUdot(nCoordinates);\n')
    f.write('\tknownUdot.setToZero();\n')
    f.write('\tfor (int i = 0; i < nCoordinates; ++i) knownUdot[i] = ua[i];\n')
    f.write('\t/// Calculate residual forces.\n')
    f.write('\tVector residualMobilityForces(nCoordinates);\n')
    f.write('\tresidualMobilityForces.setToZero();\n')
    f.write('\tmodel->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,\n')
    f.write('\t\t\tappliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);\n\n')
    
    if exportMarkerPositions:
        f.write('\t/// Marker positions.\n')
        for marker in response_markers:
            if "." in marker:
                marker_adj = marker.replace(".", "_")
            else:
                marker_adj = marker                       
            f.write('\tVec3 %s_location = %s->getLocationInGround(*state);\n' % (marker_adj, marker_adj))
        f.write('\n')
        
    if generate_pp:
        f.write('\t/// Ground reaction forces and moments.\n')
        f.write('\tSpatialVec GRF_r;\n')
        f.write('\tSpatialVec GRF_l;\n')
        f.write('\tSpatialVec GRM_r;\n')
        f.write('\tSpatialVec GRM_l;\n')
        f.write('\tVec3 normal(0, 1, 0);\n')
        count = 0
        geo1_frameNames = []
        for i in range(forceSet.getSize()):        
            c_force_elt = forceSet.get(i)  
            if c_force_elt.getConcreteClassName() == "SmoothSphereHalfSpaceForce":
                c_force_elt_name = c_force_elt.getName() 
                socket1Name = c_force_elt.getSocketNames()[1]
                socket1 = c_force_elt.getSocket(socket1Name)
                socket1_obj = socket1.getConnecteeAsObject()
                socket1_objName = socket1_obj.getName()            
                geo1 = geometrySet.get(socket1_objName)
                geo1_frameName = geo1.getFrame().getName() 

                if not geo1_frameName in geo1_frameNames:
                    f.write('\tSimTK::Transform TR_GB_%s = %s->getMobilizedBody().getBodyTransform(*state);;\n' % (geo1_frameName, geo1_frameName))    
                    geo1_frameNames.append(geo1_frameName)
                    
                f.write('\tVec3 %s_location_G = %s->findStationLocationInGround(*state, %s_location);\n' % (c_force_elt_name, geo1_frameName, c_force_elt_name))                
               	f.write('\tVec3 %s_locationCP_G = %s_location_G - %s_radius * normal;\n' % (c_force_elt_name, c_force_elt_name, c_force_elt_name))
               	f.write('\tVec3 locationCP_G_adj_%i = %s_locationCP_G - 0.5*%s_locationCP_G[1] * normal;\n' % (count, c_force_elt_name, c_force_elt_name))
               	f.write('\tVec3 %s_locationCP_B = model->getGround().findStationLocationInAnotherFrame(*state, locationCP_G_adj_%i, *%s);\n' % (c_force_elt_name, count, geo1_frameName))
                f.write('\tVec3 GRM_%i = (TR_GB_%s*%s_locationCP_B) %% GRF_%s[1];\n' % (count, geo1_frameName, c_force_elt_name, str(count)))
                
                if c_force_elt_name[-2:] == "_r":
                    f.write('\tGRF_r += GRF_%s;\n'  % (str(count)))
                    f.write('\tGRM_r += GRM_%i;\n'  % (count))   
                elif c_force_elt_name[-2:] == "_l":
                    f.write('\tGRF_l += GRF_%s;\n'  % (str(count)))  
                    f.write('\tGRM_l += GRM_%i;\n'  % (count))   
                else:
                    raise ValueError("Cannot identify contact side")
                f.write('\n')                   
                count += 1
        f.write('\n')
    
    f.write('\t/// Residual forces.\n')
    f.write('\t/// OpenSim and Simbody have different state orders so we need to adjust\n')
    f.write('\tauto indicesSimbodyInOS = getIndicesSimbodyInOS(*model);\n')
    f.write('\tfor (int i = 0; i < NU; ++i) res[0][i] =\n')
    f.write('\t\t\tvalue<T>(residualMobilityForces[indicesSimbodyInOS[i]]);\n')
    
    if exportMarkerPositions:
        f.write('\t/// Marker positions.\n')
        f.write('\tint nc = 3;\n')
        for count, marker in enumerate(response_markers):
            if "." in marker:
                marker_adj = marker.replace(".", "_")
            else:
                marker_adj = marker  
            f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(%s_location[i]);\n' % (count, marker_adj))
        count_acc = count
    f.write('\n')
    
    if generate_pp:
        f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRF_r[1][i]);\n' % (count_acc + 1))
        f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRF_l[1][i]);\n' % (count_acc + 2))
        f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRM_r[1][i]);\n' % (count_acc + 3))
        f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRM_r[1][i]);\n' % (count_acc + 4))
        f.write('\n')
        count_acc += 4
        count = 0
        for i in range(forceSet.getSize()):        
            c_force_elt = forceSet.get(i)  
            if c_force_elt.getConcreteClassName() == "SmoothSphereHalfSpaceForce":
                
                f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRF_%i[1][i]);\n' % (count_acc + 1, count))
               	f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(locationCP_G_adj_%i[i]);\n' % (count_acc + 2, count))
                f.write('\n')
                   
               	# f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRF_6_l[1][i]);\n' % (count_acc + 1))
               	# f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(contactPointpos_InGround_HC_s6_l_adj[i]);\n' % (count_acc + 1))
                count_acc += 2
                count += 1
        
    
    
    
    f.write('\treturn 0;\n')
    f.write('}\n\n')
    
    f.write('int main() {\n')
    f.write('\tRecorder x[NX];\n')
    f.write('\tRecorder u[NU];\n')
    f.write('\tRecorder tau[NR];\n')
    f.write('\tfor (int i = 0; i < NX; ++i) x[i] <<= 0;\n')
    f.write('\tfor (int i = 0; i < NU; ++i) u[i] <<= 0;\n')
    f.write('\tconst Recorder* Recorder_arg[n_in] = { x,u };\n')
    f.write('\tRecorder* Recorder_res[n_out] = { tau };\n')
    f.write('\tF_generic<Recorder>(Recorder_arg, Recorder_res);\n')
    f.write('\tdouble res[NR];\n')
    f.write('\tfor (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];\n')
    f.write('\tRecorder::stop_recording();\n')
    f.write('\treturn 0;\n')
    f.write('}\n')
    
# %% Build external function and generate DLL automatically
# TODO
    
# # %% Verification
# if verifyID:    
#     # Run ID with the .osim file
#     pathGenericTemplates = os.path.join(baseDir, "opensimPipeline",
#                                         "genericTemplates") 
#     pathGenericIDFolder = os.path.join(pathGenericTemplates, "ID")
#     pathGenericIDSetupFile = os.path.join(pathGenericIDFolder, "SetupID.xml")
    
#     idTool = opensim.InverseDynamicsTool(pathGenericIDSetupFile)
#     idTool.setName("ID_withOsimAndIDTool")
#     idTool.setModelFileName(pathModel)
#     idTool.setResultsDir(pathOutputExternalFunctionFolder)
#     idTool.setCoordinatesFileName(os.path.join(pathGenericIDFolder,
#                                                "DefaultPosition.mot"))
#     idTool.setOutputGenForceFileName("ID_withOsimAndIDTool.sto")       
#     pathSetupID = os.path.join(pathOutputExternalFunctionFolder, "SetupID.xml")
#     idTool.printToXML(pathSetupID)
    
#     command = 'opensim-cmd' + ' run-tool ' + pathSetupID
#     os.system(command)
    
#     # Extract torques from .osim + ID tool.    
#     headers = []
#     for coord in range(nCoordinates):
        
#         if (coordinateSet.get(coord).getName() == "pelvis_tx" or 
#             coordinateSet.get(coord).getName() == "pelvis_ty" or 
#             coordinateSet.get(coord).getName() == "pelvis_tz"):
#             suffix_header = "_force"
#         else:
#             suffix_header = "_moment"
#         headers.append(coordinateSet.get(coord).getName() + suffix_header)
        
#     from utils import storage2df    
#     ID_osim_df = storage2df(os.path.join(pathOutputExternalFunctionFolder,
#                                   "ID_withOsimAndIDTool.sto"), headers)
#     ID_osim = np.zeros((nCoordinates))
#     for count, coordinateOrder in enumerate(coordinatesOrder):
#         if (coordinateOrder == "pelvis_tx" or 
#             coordinateOrder == "pelvis_ty" or 
#             coordinateOrder == "pelvis_tz"):
#             suffix_header = "_force"
#         else:
#             suffix_header = "_moment"
#         ID_osim[count] = ID_osim_df.iloc[0][coordinateOrder + suffix_header]
    
#     # Extract torques from external function
#     import casadi as ca
#     os.chdir(pathOutputExternalFunctionFolder)
#     F = ca.external('F', outputModelFileName + '.dll') 
#     os.chdir(scriptDir)
    
#     vec1 = np.zeros((len(headers)*2, 1))
#     vec1[::2, :] = -1     
#     vec2 = np.zeros((len(headers), 1))
#     vec3 = np.concatenate((vec1,vec2))
#     ID_F = (F(vec3)).full().flatten()[:nCoordinates]  
    
#     assert(np.max(np.abs(ID_osim - ID_F)) < 1e-6), "error F vs ID tool & osim" 
