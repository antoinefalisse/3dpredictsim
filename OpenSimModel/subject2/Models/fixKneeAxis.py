import os
# import sys
# sys.path.append("..") # utilities in parent directory
# import dataman
import opensim

# %% Paths.
scriptDir = os.getcwd()
modelDir = os.path.dirname(scriptDir) 
OpenSimModelDir = os.path.dirname(modelDir)

# %% User settings.

pathGenericTemplates = os.path.join(OpenSimModelDir, "generic_templates")
modelName = "subject2_withMTP_weldRadius_scaled"
pathModel = os.path.join(scriptDir, modelName + ".osim")
pathOutputModelFolder = scriptDir
pathOutputModel = os.path.join(scriptDir, modelName + "_FK.osim")



# # %% Add contact spheres to the scaled model.
# # The parameters of the foot-ground contacts are based on previous work. We
# # scale the contact sphere locations based on foot dimensions.
# if addFootGroundContacts:
#     import numpy as np
        
#     # Reference values.
#     reference_contact_spheres = {
#         "s1_r": {"radius": 0.032320, "location": np.array([-0.00042152, -0.01, -0.0049972]), "orientation": np.array([0, 0, 0]), "socket_frame": "calcn_r"},
#         "s2_r": {"radius": 0.032320, "location": np.array([0.06       , -0.01, 0.020001  ]), "orientation": np.array([0, 0, 0]), "socket_frame": "calcn_r"},
#         "s3_r": {"radius": 0.023374, "location": np.array([0.165      , -0.01, 0.021183  ]), "orientation": np.array([0, 0, 0]), "socket_frame": "calcn_r"},
#         "s4_r": {"radius": 0.020508, "location": np.array([0.165      , -0.01, -0.01     ]), "orientation": np.array([0, 0, 0]), "socket_frame": "calcn_r"},
#         "s5_r": {"radius": 0.016244, "location": np.array([0.053154   , -0.01, -0.0034173]), "orientation": np.array([0, 0, 0]), "socket_frame": "toes_r" },
#         "s6_r": {"radius": 0.018414, "location": np.array([1.7381e-06 , -0.01, 0.022294  ]), "orientation": np.array([0, 0, 0]), "socket_frame": "toes_r" },
#         "s1_l": {"radius": 0.032320, "location": np.array([-0.00042152, -0.01, 0.0049972 ]), "orientation": np.array([0, 0, 0]), "socket_frame": "calcn_l"},
#         "s2_l": {"radius": 0.032320, "location": np.array([0.06       , -0.01, -0.020001 ]), "orientation": np.array([0, 0, 0]), "socket_frame": "calcn_l"},
#         "s3_l": {"radius": 0.023374, "location": np.array([0.165      , -0.01, -0.021183 ]), "orientation": np.array([0, 0, 0]), "socket_frame": "calcn_l"},
#         "s4_l": {"radius": 0.020508, "location": np.array([0.165      , -0.01, 0.01      ]), "orientation": np.array([0, 0, 0]), "socket_frame": "calcn_l"},
#         "s5_l": {"radius": 0.016244, "location": np.array([0.053154   , -0.01, 0.0034173 ]), "orientation": np.array([0, 0, 0]), "socket_frame": "toes_l" },
#         "s6_l": {"radius": 0.018414, "location": np.array([1.7381e-06 , -0.01, -0.022294 ]), "orientation": np.array([0, 0, 0]), "socket_frame": "toes_l" }}
#     reference_scale_factors = {"calcn_r": np.array([0.91392399999999996, 0.91392399999999996, 0.91392399999999996]),
#                                "toes_r":  np.array([0.91392399999999996, 0.91392399999999996, 0.91392399999999996]),
#                                "calcn_l": np.array([0.91392399999999996, 0.91392399999999996, 0.91392399999999996]),
#                                "toes_l":  np.array([0.91392399999999996, 0.91392399999999996, 0.91392399999999996])}
#     reference_contact_half_space = {"name": "floor", "location": np.array([0, 0, 0]),"orientation": np.array([0, 0, -np.pi/2]), "frame": "ground"}
#     stiffness = 1000000
#     dissipation = 2.0
#     static_friction = 0.8
#     dynamic_friction = 0.8
#     viscous_friction = 0.5
#     transition_velocity = 0.2
    
    
#     # Add contact spheres and SmoothSphereHalfSpaceForce.
#     model = opensim.Model(pathOutputFiles + ".osim")   
#     bodySet = model.get_BodySet()
#     forceSet = model.get_ForceSet()     
#     geometrySet = model.get_ContactGeometrySet()    
#     # ContactHalfSpace.
#     if reference_contact_half_space["frame"] == "ground":
#         contact_half_space_frame = model.get_ground()
#     else:
#         raise ValueError('Not yet supported.')    
#     contactHalfSpace = opensim.ContactHalfSpace(opensim.Vec3(reference_contact_half_space["location"]),
#                                                 opensim.Vec3(reference_contact_half_space["orientation"]),
#                                                 contact_half_space_frame,
#                                                 reference_contact_half_space["name"])
#     contactHalfSpace.connectSocket_frame(contact_half_space_frame)
#     model.addContactGeometry(contactHalfSpace)
#     # ContactSpheres and SmoothSphereHalfSpaceForce    
#     for ref_contact_sphere in reference_contact_spheres:    
#         # ContactSpheres
#         body = bodySet.get(reference_contact_spheres[ref_contact_sphere]["socket_frame"])
#         # Scale location based on attached_geometry scale_factors.      
#         # We don't scale the y_position
#         attached_geometry = body.get_attached_geometry(0)
#         c_scale_factors = attached_geometry.get_scale_factors().to_numpy() 
#         c_ref_scale_factors = reference_scale_factors[reference_contact_spheres[ref_contact_sphere]["socket_frame"]]
#         scale_factors = c_ref_scale_factors / c_scale_factors        
#         scale_factors[1] = 1        
#         scaled_location = reference_contact_spheres[ref_contact_sphere]["location"] / scale_factors
#         c_contactSphere = opensim.ContactSphere(reference_contact_spheres[ref_contact_sphere]["radius"],
#                                                 opensim.Vec3(scaled_location),
#                                                 body,
#                                                 ref_contact_sphere)
#         c_contactSphere.connectSocket_frame(body)
#         model.addContactGeometry(c_contactSphere)
        
#         # SmoothSphereHalfSpaceForce
#         SmoothSphereHalfSpaceForce = opensim.SmoothSphereHalfSpaceForce(
#             "SmoothSphereHalfSpaceForce_" + ref_contact_sphere, c_contactSphere, contactHalfSpace)
#         SmoothSphereHalfSpaceForce.set_stiffness(stiffness)
#         SmoothSphereHalfSpaceForce.set_dissipation(dissipation)
#         SmoothSphereHalfSpaceForce.set_static_friction(static_friction)
#         SmoothSphereHalfSpaceForce.set_dynamic_friction(dynamic_friction)
#         SmoothSphereHalfSpaceForce.set_viscous_friction(viscous_friction)
#         SmoothSphereHalfSpaceForce.set_transition_velocity(transition_velocity)        
#         SmoothSphereHalfSpaceForce.connectSocket_half_space(contactHalfSpace)
#         SmoothSphereHalfSpaceForce.connectSocket_sphere(c_contactSphere)
#         model.addForce(SmoothSphereHalfSpaceForce)
        
#     model.finalizeConnections
#     model.initSystem()
#     model.printToXML(pathOutputFiles + "_contacts.osim")

# %% Fix knee axis.

        
# Step 1: Run point kinematics with the model in its nominal pose,
# and extract the position of the origin of the tibia in the femur frame.
import glob
from utils import storage2df

pathGenericSetup = os.path.join(pathGenericTemplates, "PK")   
pathGenericSetupFile = os.path.join(pathGenericSetup,
                                    "SetupPointKinematics.xml")    
ATool = opensim.AnalyzeTool(pathGenericSetupFile, False)    
# if fixKneeAxis_model == "contacts":
#     suffix_model = "_contacts"
# else:
#     suffix_model = ""
# pathModel = pathOutputFiles + suffix_model + ".osim"        
ATool.setModelFilename(pathModel)     
ATool.setName("PK_" + modelName)
pathOutputPKFolder = os.path.join(pathOutputModelFolder, "PK")
if not os.path.exists(pathOutputPKFolder):
    os.makedirs(pathOutputPKFolder)        
ATool.setResultsDir(pathOutputPKFolder)    
pathMotionFile = os.path.join(pathGenericSetup, "DefaultPosition.mot") 
ATool.setCoordinatesFileName(pathMotionFile)
pathSetupFile =  os.path.join(pathOutputPKFolder, "SetupPK_" + modelName + ".xml")
ATool.printToXML(pathSetupFile)
command = 'opensim-cmd' + ' run-tool ' + pathSetupFile
os.system(command)    
# Remove non-necessary files.
for CleanUp in glob.glob(pathOutputPKFolder + '/*.*'):
    if ((not CleanUp.endswith(".xml")) and 
        (not CleanUp.endswith("_pos.sto"))):    
        os.remove(CleanUp)            
# Extract position
headers_PK = ["state_0", "state_1", "state_2"]
tibia_in_femur_pos = storage2df(
    os.path.join(pathOutputPKFolder, 
                 "PK_" + modelName + 
                 "_PointKinematics_origin_r_pos.sto"), headers_PK)

# Step 2: Adjust the knee joint definition, repalcing the splines by
# constant values(0).
model = opensim.Model(pathModel)  
model.finalizeConnections
model.initSystem()
jointSet = model.get_JointSet()    
knees = ["knee_r", "knee_l"]
for knee in knees:
    # Step 2a: Adjust the translation of the knee in the femur frame.
    knee_joint = jointSet.get(knee)
    femur_frame = knee_joint.get_frames(0)
    femur_frame.set_translation(
        opensim.Vec3(tibia_in_femur_pos.iloc[0].to_numpy()[1::]))

    # Step 2b: Remove the spline-based translations.
    cObj = opensim.CustomJoint.safeDownCast(knee_joint)    
    spatialtransform = cObj.get_SpatialTransform()
    
    tr1 = spatialtransform.upd_translation1()
    tr1.setCoordinateNames(opensim.ArrayStr(""))    
    tr1.set_function(opensim.Constant(0))
    
    tr2 = spatialtransform.upd_translation2()
    tr2.setCoordinateNames(opensim.ArrayStr(""))    
    tr2.set_function(opensim.Constant(0))
    
    tr3 = spatialtransform.upd_translation3()
    tr3.setCoordinateNames(opensim.ArrayStr(""))    
    tr3.set_function(opensim.Constant(0))    
    
model.printToXML(pathOutputModel)
