/*  This code describes the OpenSim model and the skeleton dynamics
    Author: Antoine Falisse
    Contributors: Joris Gillis, Gil Serrancoli, Chris Dembia
*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce_smooth.h>
#include <recorder.hpp>

#include <iostream>
#include <iterator>
#include <random>
#include <cassert>
#include <algorithm>
#include <vector>
#include <fstream>

using namespace SimTK;
using namespace OpenSim;

/*  The function F describes the OpenSim model and, implicitly, the skeleton
    dynamics. F takes as inputs joint positions and velocities (states x),
    joint accelerations (controls u), contact forces as well as several
    parameters related to the contact models (parameters p), and returns the
    joint torques as well as several variables for use in the
    optimal control problems. F is templatized using type T. F(x,u,p)->(r).
*/

// Inputs/outputs of function F
/// number of vectors in inputs/outputs of function F
constexpr int n_in = 3;
constexpr int n_out = 1;
/// number of elements in input/output vectors of function F
constexpr int ndof = 29;        // # degrees of freedom (excluding locked)
constexpr int ndofr = 31;       // # degrees of freedom (including locked)
constexpr int NX = ndof*2;      // # states
constexpr int NU = ndof;        // # controls
constexpr int NP = 54;          // # parameters
constexpr int NR = ndof+6+6;    // # residual torques + # GRFs + # GRMs

// Helper function value
template<typename T>
T value(const Recorder& e) { return e; }
template<>
double value(const Recorder& e) { return e.getValue(); }

// OpenSim and Simbody use different indices for the states/controls when the
// kinematic chain has joints up and down the origin (e.g., lumbar joint/arms
// and legs with pelvis as origin).
// The two following functions allow getting the indices from one reference
// system to the other. These functions are inspired from
// createSystemYIndexMap() in Moco.
// getIndicesOSInSimbody() returns the indices of the OpenSim Qs in the Simbody
// reference system. Note that we only care about the order here so we divide
// by 2 because the states include both Qs and Qdots.
SimTK::Array_<int> getIndicesOSInSimbody(const Model& model) {
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    SimTK::Array_<int> idxOSInSimbody(s.getNQ());
    s.updQ() = 0;
    for (int iy = 0; iy < s.getNQ(); ++iy) {
        s.updQ()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                s.updQ()[iy] = 0;
                idxOSInSimbody[iy] = isv/2;
                break;
            }
        }
    }
    return idxOSInSimbody;
}
// getIndicesSimbodyInOS() returns the indices of the Simbody Qs in the OpenSim
// reference system.
SimTK::Array_<int> getIndicesSimbodyInOS(const Model& model) {
    auto idxOSInSimbody = getIndicesOSInSimbody(model);
    auto s = model.getWorkingState();
    SimTK::Array_<int> idxSimbodyInOS(s.getNQ());
	for (int iy = 0; iy < s.getNQ(); ++iy) {
		for (int iyy = 0; iyy < s.getNQ(); ++iyy) {
			if (idxOSInSimbody[iyy] == iy) {
				idxSimbodyInOS[iy] = iyy;
				break;
			}
		}
	}
    return idxSimbodyInOS;
}

// Function F
template<typename T>
int F_generic(const T** arg, T** res) {

    // OpenSim model: create components
    /// Model
    OpenSim::Model* model;
    /// Bodies
    OpenSim::Body* pelvis;
    OpenSim::Body* femur_r;
    OpenSim::Body* femur_l;
    OpenSim::Body* tibia_r;
    OpenSim::Body* tibia_l;
    OpenSim::Body* talus_r;
    OpenSim::Body* talus_l;
    OpenSim::Body* calcn_r;
    OpenSim::Body* calcn_l;
    OpenSim::Body* toes_r;
    OpenSim::Body* toes_l;
    OpenSim::Body* torso;
    OpenSim::Body* humerus_r;
    OpenSim::Body* humerus_l;
    OpenSim::Body* ulna_r;
    OpenSim::Body* ulna_l;
    OpenSim::Body* radius_r;
    OpenSim::Body* radius_l;
    OpenSim::Body* hand_r;
    OpenSim::Body* hand_l;
    /// Joints
    OpenSim::CustomJoint* ground_pelvis;
    OpenSim::CustomJoint* hip_r;
    OpenSim::CustomJoint* hip_l;
    OpenSim::CustomJoint* knee_r;
    OpenSim::CustomJoint* knee_l;
    OpenSim::CustomJoint* ankle_r;
    OpenSim::CustomJoint* ankle_l;
    OpenSim::CustomJoint* subtalar_r;
    OpenSim::CustomJoint* subtalar_l;
    OpenSim::WeldJoint* mtp_r;
    OpenSim::WeldJoint* mtp_l;
    OpenSim::CustomJoint* back;
    OpenSim::CustomJoint* shoulder_r;
    OpenSim::CustomJoint* shoulder_l;
    OpenSim::CustomJoint* elbow_r;
    OpenSim::CustomJoint* elbow_l;
    OpenSim::CustomJoint* radioulnar_r;
    OpenSim::CustomJoint* radioulnar_l;
    OpenSim::WeldJoint* radius_hand_r;
    OpenSim::WeldJoint* radius_hand_l;
    // OpenSim model: initialize components
    /// Model
    model = new OpenSim::Model();
    /// Body specifications
    pelvis = new OpenSim::Body("pelvis", 8.84259166189724, Vec3(-0.0682778, 0, 0), Inertia(0.0741799006400181, 0.0741799006400181, 0.0405455944309864, 0, 0, 0));
    femur_l = new OpenSim::Body("femur_l", 6.98382288222561, Vec3(0, -0.170467, 0), Inertia(0.101089610270247, 0.0264992182261812, 0.106600843690507, 0, 0, 0));
    femur_r = new OpenSim::Body("femur_r", 6.98382288222561, Vec3(0, -0.170467, 0), Inertia(0.101089610270247, 0.0264992182261812, 0.106600843690507, 0, 0, 0));
    tibia_l = new OpenSim::Body("tibia_l", 2.78372323906632, Vec3(0, -0.180489, 0), Inertia(0.0353661477848549, 0.00357871733537223, 0.0358573442818668, 0, 0, 0));
    tibia_r = new OpenSim::Body("tibia_r", 2.78372323906632, Vec3(0, -0.180489, 0), Inertia(0.0353661477848549, 0.00357871733537223, 0.0358573442818668, 0, 0, 0));
    talus_l = new OpenSim::Body("talus_l", 0.0750835667988218, Vec3(0, 0, 0), Inertia(0.00062714132461258, 0.00062714132461258, 0.00062714132461258, 0, 0, 0));
    talus_r = new OpenSim::Body("talus_r", 0.0750835667988218, Vec3(0, 0, 0), Inertia(0.00062714132461258, 0.00062714132461258, 0.00062714132461258, 0, 0, 0));
    calcn_l = new OpenSim::Body("calcn_l", 0.938544584985273, Vec3(0.0913924, 0.0274177, 0), Inertia(0.000877997854457612, 0.00244585116598906, 0.00257127943091158, 0, 0, 0));
    calcn_r = new OpenSim::Body("calcn_r", 0.938544584985273, Vec3(0.0913924, 0.0274177, 0), Inertia(0.000877997854457612, 0.00244585116598906, 0.00257127943091158, 0, 0, 0));
    toes_l = new OpenSim::Body("toes_l", 0.162631005686248, Vec3(0.0316218, 0.00548355, 0.0159937), Inertia(6.2714132461258e-005, 0.000125428264922516, 6.2714132461258e-005, 0, 0, 0));
    toes_r = new OpenSim::Body("toes_r", 0.162631005686248, Vec3(0.0316218, 0.00548355, -0.0159937), Inertia(6.2714132461258e-005, 0.000125428264922516, 6.2714132461258e-005, 0, 0, 0));
    torso = new OpenSim::Body("torso", 25.7060604306454, Vec3(-0.0267603, 0.306505, 0), Inertia(0.981166155448334, 0.451354452950527, 0.981166155448334, 0, 0, 0));
    humerus_l = new OpenSim::Body("humerus_l", 1.52611854532613, Vec3(0, -0.169033, 0), Inertia(0.00947044247669374, 0.00326700932918591, 0.0106302664632502, 0, 0, 0));
    humerus_r = new OpenSim::Body("humerus_r", 1.52611854532613, Vec3(0, -0.169033, 0), Inertia(0.00947044247669374, 0.00326700932918591, 0.0106302664632502, 0, 0, 0));
    ulna_l = new OpenSim::Body("ulna_l", 0.456132668302843, Vec3(0, -0.118275, 0), Inertia(0.00214171997483337, 0.000446854471454093, 0.00232320941226861, 0, 0, 0));
    ulna_r = new OpenSim::Body("ulna_r", 0.456132668302843, Vec3(0, -0.118275, 0), Inertia(0.00214171997483337, 0.000446854471454093, 0.00232320941226861, 0, 0, 0));
    radius_l = new OpenSim::Body("radius_l", 0.456132668302843, Vec3(0, -0.118275, 0), Inertia(0.00214171997483337, 0.000446854471454093, 0.00232320941226861, 0, 0, 0));
    radius_r = new OpenSim::Body("radius_r", 0.456132668302843, Vec3(0, -0.118275, 0), Inertia(0.00214171997483337, 0.000446854471454093, 0.00232320941226861, 0, 0, 0));
    hand_l = new OpenSim::Body("hand_l", 0.34350731810461, Vec3(0, -0.0668239, 0), Inertia(0.000644974415108496, 0.000395516821821017, 0.000968907753638324, 0, 0, 0));
    hand_r = new OpenSim::Body("hand_r", 0.34350731810461, Vec3(0, -0.0668239, 0), Inertia(0.000644974415108496, 0.000395516821821017, 0.000968907753638324, 0, 0, 0));
    /// Joints
    /// Ground-Pelvis transform
    SpatialTransform st_ground_pelvis;
    st_ground_pelvis[0].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tilt", 1, 1));
    st_ground_pelvis[0].setFunction(new LinearFunction());
    st_ground_pelvis[0].setAxis(Vec3(0, 0, 1));
    st_ground_pelvis[1].setCoordinateNames(OpenSim::Array<std::string>("pelvis_list", 1, 1));
    st_ground_pelvis[1].setFunction(new LinearFunction());
    st_ground_pelvis[1].setAxis(Vec3(1, 0, 0));
    st_ground_pelvis[2].setCoordinateNames(OpenSim::Array<std::string>("pelvis_rotation", 1, 1));
    st_ground_pelvis[2].setFunction(new LinearFunction());
    st_ground_pelvis[2].setAxis(Vec3(0, 1, 0));
    st_ground_pelvis[3].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tx", 1, 1));
    st_ground_pelvis[3].setFunction(new LinearFunction());
    st_ground_pelvis[3].setAxis(Vec3(1, 0, 0));
    st_ground_pelvis[4].setCoordinateNames(OpenSim::Array<std::string>("pelvis_ty", 1, 1));
    st_ground_pelvis[4].setFunction(new LinearFunction());
    st_ground_pelvis[4].setAxis(Vec3(0, 1, 0));
    st_ground_pelvis[5].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tz", 1, 1));
    st_ground_pelvis[5].setFunction(new LinearFunction());
    st_ground_pelvis[5].setAxis(Vec3(0, 0, 1));
    /// Hip_l transform
    SpatialTransform st_hip_l;
    st_hip_l[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_l", 1, 1));
    st_hip_l[0].setFunction(new LinearFunction());
    st_hip_l[0].setAxis(Vec3(0, 0, 1));
    st_hip_l[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_l", 1, 1));
    st_hip_l[1].setFunction(new LinearFunction());
    st_hip_l[1].setAxis(Vec3(-1, 0, 0));
    st_hip_l[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_l", 1, 1));
    st_hip_l[2].setFunction(new LinearFunction());
    st_hip_l[2].setAxis(Vec3(0, -1, 0));
    /// Hip_r transform
    SpatialTransform st_hip_r;
    st_hip_r[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_r", 1, 1));
    st_hip_r[0].setFunction(new LinearFunction());
    st_hip_r[0].setAxis(Vec3(0, 0, 1));
    st_hip_r[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_r", 1, 1));
    st_hip_r[1].setFunction(new LinearFunction());
    st_hip_r[1].setAxis(Vec3(1, 0, 0));
    st_hip_r[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_r", 1, 1));
    st_hip_r[2].setFunction(new LinearFunction());
    st_hip_r[2].setAxis(Vec3(0, 1, 0));
    /// Knee_l transform
    SpatialTransform st_knee_l;
    st_knee_l[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
    st_knee_l[2].setFunction(new LinearFunction());
    st_knee_l[2].setAxis(Vec3(0, 0, 1));
    /// Knee_r transform
    SpatialTransform st_knee_r;
    st_knee_r[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
    st_knee_r[2].setFunction(new LinearFunction());
    st_knee_r[2].setAxis(Vec3(0, 0, 1));
    /// Ankle_l transform
    SpatialTransform st_ankle_l;
    st_ankle_l[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_l", 1, 1));
    st_ankle_l[0].setFunction(new LinearFunction());
    st_ankle_l[0].setAxis(Vec3(0.10501355, 0.17402245, 0.97912632));
    /// Ankle_r transform
    SpatialTransform st_ankle_r;
    st_ankle_r[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_r", 1, 1));
    st_ankle_r[0].setFunction(new LinearFunction());
    st_ankle_r[0].setAxis(Vec3(-0.10501355, -0.17402245, 0.97912632));
    /// Subtalar_l transform
    SpatialTransform st_subtalar_l;
    st_subtalar_l[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_l", 1, 1));
    st_subtalar_l[0].setFunction(new LinearFunction());
    st_subtalar_l[0].setAxis(Vec3(-0.78717961, -0.60474746, -0.12094949));
    /// Subtalar_r transform
    SpatialTransform st_subtalar_r;
    st_subtalar_r[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_r", 1, 1));
    st_subtalar_r[0].setFunction(new LinearFunction());
    st_subtalar_r[0].setAxis(Vec3(0.78717961, 0.60474746, -0.12094949));
    /// Back transform
    SpatialTransform st_back;
    st_back[0].setCoordinateNames(OpenSim::Array<std::string>("lumbar_extension", 1, 1));
    st_back[0].setFunction(new LinearFunction());
    st_back[0].setAxis(Vec3(0, 0, 1));
    st_back[1].setCoordinateNames(OpenSim::Array<std::string>("lumbar_bending", 1, 1));
    st_back[1].setFunction(new LinearFunction());
    st_back[1].setAxis(Vec3(1, 0, 0));
    st_back[2].setCoordinateNames(OpenSim::Array<std::string>("lumbar_rotation", 1, 1));
    st_back[2].setFunction(new LinearFunction());
    st_back[2].setAxis(Vec3(0, 1, 0));
    /// Shoulder_l transform
    SpatialTransform st_sho_l;
    st_sho_l[0].setCoordinateNames(OpenSim::Array<std::string>("arm_flex_l", 1, 1));
    st_sho_l[0].setFunction(new LinearFunction());
    st_sho_l[0].setAxis(Vec3(0, 0, 1));
    st_sho_l[1].setCoordinateNames(OpenSim::Array<std::string>("arm_add_l", 1, 1));
    st_sho_l[1].setFunction(new LinearFunction());
    st_sho_l[1].setAxis(Vec3(-1, 0, 0));
    st_sho_l[2].setCoordinateNames(OpenSim::Array<std::string>("arm_rot_l", 1, 1));
    st_sho_l[2].setFunction(new LinearFunction());
    st_sho_l[2].setAxis(Vec3(0, -1, 0));
    /// Shoulder_r transform
    SpatialTransform st_sho_r;
    st_sho_r[0].setCoordinateNames(OpenSim::Array<std::string>("arm_flex_r", 1, 1));
    st_sho_r[0].setFunction(new LinearFunction());
    st_sho_r[0].setAxis(Vec3(0, 0, 1));
    st_sho_r[1].setCoordinateNames(OpenSim::Array<std::string>("arm_add_r", 1, 1));
    st_sho_r[1].setFunction(new LinearFunction());
    st_sho_r[1].setAxis(Vec3(1, 0, 0));
    st_sho_r[2].setCoordinateNames(OpenSim::Array<std::string>("arm_rot_r", 1, 1));
    st_sho_r[2].setFunction(new LinearFunction());
    st_sho_r[2].setAxis(Vec3(0, 1, 0));
    /// Elbow_l transform
    SpatialTransform st_elb_l;
    st_elb_l[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flex_l", 1, 1));
    st_elb_l[0].setFunction(new LinearFunction());
    st_elb_l[0].setAxis(Vec3(-0.22604696, -0.022269, 0.97386183));
    /// Elbow_r transform
    SpatialTransform st_elb_r;
    st_elb_r[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flex_r", 1, 1));
    st_elb_r[0].setFunction(new LinearFunction());
    st_elb_r[0].setAxis(Vec3(0.22604696, 0.022269, 0.97386183));
    /// Radioulnar_l transform
    SpatialTransform st_radioulnar_l;
    st_radioulnar_l[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_l", 1, 1));
    st_radioulnar_l[0].setFunction(new LinearFunction());
    st_radioulnar_l[0].setAxis(Vec3(-0.05639803, -0.99840646, 0.001952));
    /// Radioulnar_r transform
    SpatialTransform st_radioulnar_r;
    st_radioulnar_r[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_r", 1, 1));
    st_radioulnar_r[0].setFunction(new LinearFunction());
    st_radioulnar_r[0].setAxis(Vec3(0.05639803, 0.99840646, 0.001952));
    /// Joint specifications
    ground_pelvis = new CustomJoint("ground_pelvis", model->getGround(), Vec3(0), Vec3(0), *pelvis, Vec3(0), Vec3(0), st_ground_pelvis);
    hip_l = new CustomJoint("hip_l", *pelvis, Vec3(-0.0682778001711179, -0.0638353973311301, -0.0823306940058688), Vec3(0), *femur_l, Vec3(0), Vec3(0), st_hip_l);
    hip_r = new CustomJoint("hip_r", *pelvis, Vec3(-0.0682778001711179, -0.0638353973311301, 0.0823306940058688), Vec3(0), *femur_r, Vec3(0), Vec3(0), st_hip_r);
    knee_l = new CustomJoint("knee_l", *femur_l, Vec3(-0.00451221232146798, -0.396907245921447, 0), Vec3(0), *tibia_l, Vec3(0), Vec3(0), st_knee_l);
    knee_r = new CustomJoint("knee_r", *femur_r, Vec3(-0.00451221232146798, -0.396907245921447, 0), Vec3(0), *tibia_r, Vec3(0), Vec3(0), st_knee_r);
    ankle_l = new CustomJoint("ankle_l", *tibia_l, Vec3(0, -0.415694825374905, 0), Vec3(0), *talus_l, Vec3(0), Vec3(0), st_ankle_l);
    ankle_r = new CustomJoint("ankle_r", *tibia_r, Vec3(0, -0.415694825374905, 0), Vec3(0), *talus_r, Vec3(0), Vec3(0), st_ankle_r);
    subtalar_l = new CustomJoint("subtalar_l", *talus_l, Vec3(-0.0445720919117321, -0.0383391276542374, -0.00723828107321956), Vec3(0), *calcn_l, Vec3(0), Vec3(0),st_subtalar_l);
    subtalar_r = new CustomJoint("subtalar_r", *talus_r, Vec3(-0.0445720919117321, -0.0383391276542374, 0.00723828107321956), Vec3(0), *calcn_r, Vec3(0), Vec3(0),st_subtalar_r);
    mtp_l = new WeldJoint("mtp_l", *calcn_l, Vec3(0.163409678774199, -0.00182784875586352, -0.000987038328166303), Vec3(0), *toes_l, Vec3(0), Vec3(0));
    mtp_r = new WeldJoint("mtp_r", *calcn_r, Vec3(0.163409678774199, -0.00182784875586352, 0.000987038328166303), Vec3(0), *toes_r, Vec3(0), Vec3(0));
    back = new CustomJoint("back", *pelvis, Vec3(-0.0972499926058214, 0.0787077894476112, 0), Vec3(0), *torso, Vec3(0), Vec3(0), st_back);
    shoulder_l = new CustomJoint("shoulder_l", *torso, Vec3(0.0028142880546385, 0.35583331053375, -0.151641511660395), Vec3(0), *humerus_l, Vec3(0), Vec3(0), st_sho_l);
    shoulder_r = new CustomJoint("shoulder_r", *torso, Vec3(0.0028142880546385, 0.35583331053375, 0.151641511660395), Vec3(0), *humerus_r, Vec3(0), Vec3(0), st_sho_r);
    elbow_l = new CustomJoint("elbow_l", *humerus_l, Vec3(0.0135060695814636, -0.294158784030305, 0.00985930748890318), Vec3(0), *ulna_l, Vec3(0), Vec3(0), st_elb_l);
    elbow_r = new CustomJoint("elbow_r", *humerus_r, Vec3(0.0135060695814636, -0.294158784030305, -0.00985930748890318), Vec3(0), *ulna_r, Vec3(0), Vec3(0), st_elb_r);
    radioulnar_l = new CustomJoint("radioulnar_l", *ulna_l, Vec3(-0.00660142656498441, -0.0127641973139218, -0.0255961065994483), Vec3(0), *radius_l, Vec3(0), Vec3(0),st_radioulnar_l);
    radioulnar_r = new CustomJoint("radioulnar_r", *ulna_r, Vec3(-0.00660142656498441, -0.0127641973139218, 0.0255961065994483), Vec3(0), *radius_r, Vec3(0), Vec3(0),st_radioulnar_r);
    radius_hand_l = new WeldJoint("radius_hand_l", *radius_l, Vec3(-0.00863278571312143, -0.231438537611489, -0.0133559410657705), Vec3(0), *hand_l, Vec3(0), Vec3(0));
    radius_hand_r = new WeldJoint("radius_hand_r", *radius_r, Vec3(-0.00863278571312143, -0.231438537611489, 0.0133559410657705), Vec3(0), *hand_r, Vec3(0), Vec3(0));
    /// Add bodies and joints to model
    model->addBody(pelvis);		    model->addJoint(ground_pelvis);
    model->addBody(femur_l);		model->addJoint(hip_l);
    model->addBody(femur_r);		model->addJoint(hip_r);
    model->addBody(tibia_l);		model->addJoint(knee_l);
    model->addBody(tibia_r);		model->addJoint(knee_r);
    model->addBody(talus_l);		model->addJoint(ankle_l);
    model->addBody(talus_r);		model->addJoint(ankle_r);
    model->addBody(calcn_l);		model->addJoint(subtalar_l);
    model->addBody(calcn_r);		model->addJoint(subtalar_r);
    model->addBody(toes_l);		    model->addJoint(mtp_l);
    model->addBody(toes_r);		    model->addJoint(mtp_r);
    model->addBody(torso);          model->addJoint(back);
    model->addBody(humerus_l);      model->addJoint(shoulder_l);
    model->addBody(humerus_r);      model->addJoint(shoulder_r);
    model->addBody(ulna_l);         model->addJoint(elbow_l);
    model->addBody(ulna_r);         model->addJoint(elbow_r);
    model->addBody(radius_l);       model->addJoint(radioulnar_l);
    model->addBody(radius_r);       model->addJoint(radioulnar_r);
    model->addBody(hand_l);         model->addJoint(radius_hand_l);
    model->addBody(hand_r);         model->addJoint(radius_hand_r);

    // Initialize system and state
    SimTK::State* state;
    state = new State(model->initSystem());

    // Read inputs
    std::vector<T> x(arg[0], arg[0] + NX);
    std::vector<T> u(arg[1], arg[1] + NU);
    std::vector<T> p(arg[2], arg[2] + NP);

    // States and controls
    T ua[NU+2]; /// joint accelerations (Qdotdots) - controls
    T up[NP]; /// contact model parameters - parameters
    Vector QsUs(NX+4); /// joint positions (Qs) and velocities (Us) - states

    // Assign inputs to model variables
    /// States
    for (int i = 0; i < NX; ++i) QsUs[i] = x[i];
    /// pro_sup dofs are locked so Qs and Qdots are hard coded
    QsUs[NX] = 1.51;
    QsUs[NX+1] = 0;
    QsUs[NX+2] = 1.51;
    QsUs[NX+3] = 0;
    /// Controls
    /// OpenSim and Simbody have different state orders so we need to adjust
    auto indicesOSInSimbody = getIndicesOSInSimbody(*model);
    for (int i = 0; i < NU; ++i) ua[i] = u[indicesOSInSimbody[i]];
    /// pro_sup dofs are locked so Qs and Qdots are hard coded
    ua[29] = 0;
    ua[30] = 0;
    /// Parameters
    for (int i = 0; i < NP; ++i) up[i] = p[i];

    // Set state variables and realize
    model->setStateVariableValues(*state, QsUs);
    model->realizeVelocity(*state);

    // Compute residual forces
    /// appliedMobilityForces (# mobilities)
    Vector appliedMobilityForces(ndofr);
    appliedMobilityForces.setToZero();
    /// appliedBodyForces (# bodies + ground)
    Vector_<SpatialVec> appliedBodyForces;
    int nbodies = model->getBodySet().getSize() + 1;
    appliedBodyForces.resize(nbodies);
    appliedBodyForces.setToZero();
    /// Set gravity
    Vec3 gravity(0);
    gravity[1] = -9.81;
    /// Add weights to appliedBodyForces
    for (int i = 0; i < model->getBodySet().getSize(); ++i) {
        model->getMatterSubsystem().addInStationForce(*state,
            model->getBodySet().get(i).getMobilizedBodyIndex(),
            model->getBodySet().get(i).getMassCenter(),
            model->getBodySet().get(i).getMass()*gravity, appliedBodyForces);
    }
    /// Extract contact forces
    Vec3 AppliedPointForce_s1_l, AppliedPointForce_s2_l;
    Vec3 AppliedPointForce_s3_l, AppliedPointForce_s4_l;
    Vec3 AppliedPointForce_s5_l, AppliedPointForce_s6_l;
    Vec3 AppliedPointForce_s1_r, AppliedPointForce_s2_r;
    Vec3 AppliedPointForce_s3_r, AppliedPointForce_s4_r;
    Vec3 AppliedPointForce_s5_r, AppliedPointForce_s6_r;
    int nc = 3;
    for (int i = 0; i < nc; ++i) {
        AppliedPointForce_s1_l[i]   = up[i];
        AppliedPointForce_s2_l[i]   = up[i + nc];
        AppliedPointForce_s3_l[i]   = up[i + nc + nc];
        AppliedPointForce_s4_l[i]   = up[i + nc + nc + nc];
        AppliedPointForce_s5_l[i]   = up[i + nc + nc + nc + nc];
        AppliedPointForce_s6_l[i]   = up[i + nc + nc + nc + nc + nc];
        AppliedPointForce_s1_r[i]   = up[i + nc + nc + nc + nc + nc + nc];
        AppliedPointForce_s2_r[i]   = up[i + nc + nc + nc + nc + nc + nc + nc];
        AppliedPointForce_s3_r[i]   = up[i + nc + nc + nc + nc + nc + nc + nc + nc];
        AppliedPointForce_s4_r[i]   = up[i + nc + nc + nc + nc + nc + nc + nc + nc + nc];
        AppliedPointForce_s5_r[i]   = up[i + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc];
        AppliedPointForce_s6_r[i]   = up[i + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc];
    }
    /// Extract contact sphere locations
    Vec3 locSphere_s1_r, locSphere_s2_r;
    Vec3 locSphere_s3_r, locSphere_s4_r;
    Vec3 locSphere_s5_r, locSphere_s6_r;
    /// Vertical positions are fixed
    locSphere_s1_r[1] = -0.021859;
    locSphere_s2_r[1] = -0.021859;
    locSphere_s3_r[1] = -0.021859;
    locSphere_s4_r[1] = -0.0214476;
    locSphere_s5_r[1] = -0.021859;
    locSphere_s6_r[1] = -0.0214476;
    int count = 0;
    for (int i = 0; i < nc; i+=2) {
        locSphere_s1_r[i]   = up[count + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc];
        locSphere_s2_r[i]   = up[count + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc-1];
        locSphere_s3_r[i]   = up[count + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc-1 + nc-1];
        locSphere_s4_r[i]   = up[count + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc-1 + nc-1 + nc-1];
        locSphere_s5_r[i]   = up[count + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc-1 + nc-1 + nc-1 + nc-1];
        locSphere_s6_r[i]   = up[count + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc-1 + nc-1 + nc-1 + nc-1 + nc-1];
        ++count;
    }
    Vec3 locSphere_s1_l(locSphere_s1_r[0],locSphere_s1_r[1],-locSphere_s1_r[2]);
    Vec3 locSphere_s2_l(locSphere_s2_r[0],locSphere_s2_r[1],-locSphere_s2_r[2]);
    Vec3 locSphere_s3_l(locSphere_s3_r[0],locSphere_s3_r[1],-locSphere_s3_r[2]);
    Vec3 locSphere_s4_l(locSphere_s4_r[0],locSphere_s4_r[1],-locSphere_s4_r[2]);
    Vec3 locSphere_s5_l(locSphere_s5_r[0],locSphere_s5_r[1],-locSphere_s5_r[2]);
    Vec3 locSphere_s6_l(locSphere_s6_r[0],locSphere_s6_r[1],-locSphere_s6_r[2]);
    /// Extract radii
    osim_double_adouble radius_s1, radius_s2, radius_s3, radius_s4, radius_s5, radius_s6;
    radius_s1 =  up[nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc-1 + nc-1 + nc-1 + nc-1 + nc-1 + nc-1];
    radius_s2 =  up[nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc-1 + nc-1 + nc-1 + nc-1 + nc-1 + nc-1 + 1];
    radius_s3 =  up[nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc-1 + nc-1 + nc-1 + nc-1 + nc-1 + nc-1 + 2];
    radius_s4 =  up[nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc-1 + nc-1 + nc-1 + nc-1 + nc-1 + nc-1 + 3];
    radius_s5 =  up[nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc-1 + nc-1 + nc-1 + nc-1 + nc-1 + nc-1 + 4];
    radius_s6 =  up[nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc + nc-1 + nc-1 + nc-1 + nc-1 + nc-1 + nc-1 + 5];
    /// Compute contact point positions in body frames
    Vec3 normal = Vec3(0, 1, 0);
    /// sphere 1 left
    Vec3 pos_InGround_HC_s1_l = calcn_l->findStationLocationInGround(*state, locSphere_s1_l);
    Vec3 contactPointpos_InGround_HC_s1_l = pos_InGround_HC_s1_l - radius_s1*normal;
    Vec3 contactPointpos_InGround_HC_s1_l_adj = contactPointpos_InGround_HC_s1_l - 0.5*contactPointpos_InGround_HC_s1_l[1]*normal;
    Vec3 contactPointPos_InBody_HC_s1_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s1_l_adj, *calcn_l);
    /// sphere 2 left
    Vec3 pos_InGround_HC_s2_l = calcn_l->findStationLocationInGround(*state, locSphere_s2_l);
    Vec3 contactPointpos_InGround_HC_s2_l = pos_InGround_HC_s2_l - radius_s2*normal;
    Vec3 contactPointpos_InGround_HC_s2_l_adj = contactPointpos_InGround_HC_s2_l - 0.5*contactPointpos_InGround_HC_s2_l[1]*normal;
    Vec3 contactPointPos_InBody_HC_s2_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s2_l_adj, *calcn_l);
    /// sphere 3 left
    Vec3 pos_InGround_HC_s3_l = calcn_l->findStationLocationInGround(*state, locSphere_s3_l);
    Vec3 contactPointpos_InGround_HC_s3_l = pos_InGround_HC_s3_l - radius_s3*normal;
    Vec3 contactPointpos_InGround_HC_s3_l_adj = contactPointpos_InGround_HC_s3_l - 0.5*contactPointpos_InGround_HC_s3_l[1]*normal;
    Vec3 contactPointPos_InBody_HC_s3_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s3_l_adj, *calcn_l);
    /// sphere 4 left
    Vec3 pos_InGround_HC_s4_l = toes_l->findStationLocationInGround(*state, locSphere_s4_l);
    Vec3 contactPointpos_InGround_HC_s4_l = pos_InGround_HC_s4_l - radius_s4*normal;
    Vec3 contactPointpos_InGround_HC_s4_l_adj = contactPointpos_InGround_HC_s4_l - 0.5*contactPointpos_InGround_HC_s4_l[1]*normal;
    Vec3 contactPointPos_InBody_HC_s4_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s4_l_adj, *toes_l);
    /// sphere 5 left
    Vec3 pos_InGround_HC_s5_l = calcn_l->findStationLocationInGround(*state, locSphere_s5_l);
    Vec3 contactPointpos_InGround_HC_s5_l = pos_InGround_HC_s5_l - radius_s5*normal;
    Vec3 contactPointpos_InGround_HC_s5_l_adj = contactPointpos_InGround_HC_s5_l - 0.5*contactPointpos_InGround_HC_s5_l[1]*normal;
    Vec3 contactPointPos_InBody_HC_s5_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s5_l_adj, *calcn_l);
    /// sphere 6 left
    Vec3 pos_InGround_HC_s6_l = toes_l->findStationLocationInGround(*state, locSphere_s6_l);
    Vec3 contactPointpos_InGround_HC_s6_l = pos_InGround_HC_s6_l - radius_s6*normal;
    Vec3 contactPointpos_InGround_HC_s6_l_adj = contactPointpos_InGround_HC_s6_l - 0.5*contactPointpos_InGround_HC_s6_l[1]*normal;
    Vec3 contactPointPos_InBody_HC_s6_l = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s6_l_adj, *toes_l);
    /// sphere 1 right
    Vec3 pos_InGround_HC_s1_r = calcn_r->findStationLocationInGround(*state, locSphere_s1_r);
    Vec3 contactPointpos_InGround_HC_s1_r = pos_InGround_HC_s1_r - radius_s1*normal;
    Vec3 contactPointpos_InGround_HC_s1_r_adj = contactPointpos_InGround_HC_s1_r - 0.5*contactPointpos_InGround_HC_s1_r[1]*normal;
    Vec3 contactPointPos_InBody_HC_s1_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s1_r_adj, *calcn_r);
    /// sphere 2 right
    Vec3 pos_InGround_HC_s2_r = calcn_r->findStationLocationInGround(*state, locSphere_s2_r);
    Vec3 contactPointpos_InGround_HC_s2_r = pos_InGround_HC_s2_r - radius_s2*normal;
    Vec3 contactPointpos_InGround_HC_s2_r_adj = contactPointpos_InGround_HC_s2_r - 0.5*contactPointpos_InGround_HC_s2_r[1]*normal;
    Vec3 contactPointPos_InBody_HC_s2_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s2_r_adj, *calcn_r);
    /// sphere 3 right
    Vec3 pos_InGround_HC_s3_r = calcn_r->findStationLocationInGround(*state, locSphere_s3_r);
    Vec3 contactPointpos_InGround_HC_s3_r = pos_InGround_HC_s3_r - radius_s3*normal;
    Vec3 contactPointpos_InGround_HC_s3_r_adj = contactPointpos_InGround_HC_s3_r - 0.5*contactPointpos_InGround_HC_s3_r[1]*normal;
    Vec3 contactPointPos_InBody_HC_s3_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s3_r_adj, *calcn_r);
    /// sphere 4 right
    Vec3 pos_InGround_HC_s4_r = toes_r->findStationLocationInGround(*state, locSphere_s4_r);
    Vec3 contactPointpos_InGround_HC_s4_r = pos_InGround_HC_s4_r - radius_s4*normal;
    Vec3 contactPointpos_InGround_HC_s4_r_adj = contactPointpos_InGround_HC_s4_r - 0.5*contactPointpos_InGround_HC_s4_r[1]*normal;
    Vec3 contactPointPos_InBody_HC_s4_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s4_r_adj, *toes_r);
    /// sphere 5 right
    Vec3 pos_InGround_HC_s5_r = calcn_r->findStationLocationInGround(*state, locSphere_s5_r);
    Vec3 contactPointpos_InGround_HC_s5_r = pos_InGround_HC_s5_r - radius_s5*normal;
    Vec3 contactPointpos_InGround_HC_s5_r_adj = contactPointpos_InGround_HC_s5_r - 0.5*contactPointpos_InGround_HC_s5_r[1]*normal;
    Vec3 contactPointPos_InBody_HC_s5_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s5_r_adj, *calcn_r);
    /// sphere 6 right
    Vec3 pos_InGround_HC_s6_r = toes_r->findStationLocationInGround(*state, locSphere_s6_r);
    Vec3 contactPointpos_InGround_HC_s6_r = pos_InGround_HC_s6_r - radius_s6*normal;
    Vec3 contactPointpos_InGround_HC_s6_r_adj = contactPointpos_InGround_HC_s6_r - 0.5*contactPointpos_InGround_HC_s6_r[1]*normal;
    Vec3 contactPointPos_InBody_HC_s6_r = model->getGround().findStationLocationInAnotherFrame(*state, contactPointpos_InGround_HC_s6_r_adj, *toes_r);
    /// Add contact forces to appliedBodyForces
    model->getMatterSubsystem().addInStationForce(*state, calcn_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s1_l, AppliedPointForce_s1_l, appliedBodyForces);
    model->getMatterSubsystem().addInStationForce(*state, calcn_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s2_l, AppliedPointForce_s2_l, appliedBodyForces);
    model->getMatterSubsystem().addInStationForce(*state, calcn_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s3_l, AppliedPointForce_s3_l, appliedBodyForces);
    model->getMatterSubsystem().addInStationForce(*state, toes_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s4_l, AppliedPointForce_s4_l, appliedBodyForces);
    model->getMatterSubsystem().addInStationForce(*state, calcn_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s5_l, AppliedPointForce_s5_l, appliedBodyForces);
    model->getMatterSubsystem().addInStationForce(*state, toes_l->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s6_l, AppliedPointForce_s6_l, appliedBodyForces);
    model->getMatterSubsystem().addInStationForce(*state, calcn_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s1_r, AppliedPointForce_s1_r, appliedBodyForces);
    model->getMatterSubsystem().addInStationForce(*state, calcn_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s2_r, AppliedPointForce_s2_r, appliedBodyForces);
    model->getMatterSubsystem().addInStationForce(*state, calcn_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s3_r, AppliedPointForce_s3_r, appliedBodyForces);
    model->getMatterSubsystem().addInStationForce(*state, toes_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s4_r, AppliedPointForce_s4_r, appliedBodyForces);
    model->getMatterSubsystem().addInStationForce(*state, calcn_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s5_r, AppliedPointForce_s5_r, appliedBodyForces);
    model->getMatterSubsystem().addInStationForce(*state, toes_r->getMobilizedBodyIndex(), contactPointPos_InBody_HC_s6_r, AppliedPointForce_s6_r, appliedBodyForces);
    /// knownUdot
    Vector knownUdot(ndofr);
    knownUdot.setToZero();
    for (int i = 0; i < ndofr; ++i) knownUdot[i] = ua[i];
    ///  Calculate residual forces
    Vector residualMobilityForces(ndof);
    residualMobilityForces.setToZero();
    model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,
        appliedMobilityForces, appliedBodyForces, knownUdot,
        residualMobilityForces);

    // Compute contact torques about the ground frame origin
    /// Get transforms
    SimTK::Transform TR_GB_calcn_l = calcn_l->getMobilizedBody().getBodyTransform(*state);
    SimTK::Transform TR_GB_calcn_r = calcn_r->getMobilizedBody().getBodyTransform(*state);
    SimTK::Transform TR_GB_toes_l = toes_l->getMobilizedBody().getBodyTransform(*state);
    SimTK::Transform TR_GB_toes_r = toes_r->getMobilizedBody().getBodyTransform(*state);
    /// Calculate torques
    Vec3 AppliedPointTorque_s1_l, AppliedPointTorque_s2_l, AppliedPointTorque_s3_l, AppliedPointTorque_s4_l, AppliedPointTorque_s5_l, AppliedPointTorque_s6_l;
    Vec3 AppliedPointTorque_s1_r, AppliedPointTorque_s2_r, AppliedPointTorque_s3_r, AppliedPointTorque_s4_r, AppliedPointTorque_s5_r, AppliedPointTorque_s6_r;
    AppliedPointTorque_s1_l = (TR_GB_calcn_l*contactPointPos_InBody_HC_s1_l) % AppliedPointForce_s1_l;
    AppliedPointTorque_s2_l = (TR_GB_calcn_l*contactPointPos_InBody_HC_s2_l) % AppliedPointForce_s2_l;
    AppliedPointTorque_s3_l = (TR_GB_calcn_l*contactPointPos_InBody_HC_s3_l) % AppliedPointForce_s3_l;
    AppliedPointTorque_s4_l = (TR_GB_toes_l*contactPointPos_InBody_HC_s4_l) % AppliedPointForce_s4_l;
    AppliedPointTorque_s5_l = (TR_GB_calcn_l*contactPointPos_InBody_HC_s5_l) % AppliedPointForce_s5_l;
    AppliedPointTorque_s6_l = (TR_GB_toes_l*contactPointPos_InBody_HC_s6_l) % AppliedPointForce_s6_l;
    AppliedPointTorque_s1_r = (TR_GB_calcn_r*contactPointPos_InBody_HC_s1_r) % AppliedPointForce_s1_r;
    AppliedPointTorque_s2_r = (TR_GB_calcn_r*contactPointPos_InBody_HC_s2_r) % AppliedPointForce_s2_r;
    AppliedPointTorque_s3_r = (TR_GB_calcn_r*contactPointPos_InBody_HC_s3_r) % AppliedPointForce_s3_r;
    AppliedPointTorque_s4_r = (TR_GB_toes_r*contactPointPos_InBody_HC_s4_r) % AppliedPointForce_s4_r;
    AppliedPointTorque_s5_r = (TR_GB_calcn_r*contactPointPos_InBody_HC_s5_r) % AppliedPointForce_s5_r;
    AppliedPointTorque_s6_r = (TR_GB_toes_r*contactPointPos_InBody_HC_s6_r) % AppliedPointForce_s6_r;
    /// Contact torques
    Vec3 MOM_l, MOM_r;
    MOM_l = AppliedPointTorque_s1_l + AppliedPointTorque_s2_l + AppliedPointTorque_s3_l + AppliedPointTorque_s4_l + AppliedPointTorque_s5_l + AppliedPointTorque_s6_l;
    MOM_r = AppliedPointTorque_s1_r + AppliedPointTorque_s2_r + AppliedPointTorque_s3_r + AppliedPointTorque_s4_r + AppliedPointTorque_s5_r + AppliedPointTorque_s6_r;
    /// Contact forces
    Vec3 GRF_r = AppliedPointForce_s1_r + AppliedPointForce_s2_r + AppliedPointForce_s3_r + AppliedPointForce_s4_r + AppliedPointForce_s5_r + AppliedPointForce_s6_r;
    Vec3 GRF_l = AppliedPointForce_s1_l + AppliedPointForce_s2_l + AppliedPointForce_s3_l + AppliedPointForce_s4_l + AppliedPointForce_s5_l + AppliedPointForce_s6_l;

    // Extract results
    /// Residual forces
    /// OpenSim and Simbody have different state orders so we need to adjust
    auto indicesSimbodyInOS = getIndicesSimbodyInOS(*model);
    for (int i = 0; i < NU; ++i) res[0][i] =
            value<T>(residualMobilityForces[indicesSimbodyInOS[i]]);
    /// Contact forces
    for (int i = 0; i < nc; ++i) {
        res[0][i + ndof] = value<T>(GRF_r[i]);      /// GRF_r
    }
    for (int i = 0; i < nc; ++i) {
        res[0][i + ndof + nc] = value<T>(GRF_l[i]); /// GRF_l
    }
    /// Contact torques
    for (int i = 0; i < nc; ++i) {
        res[0][i + ndof + nc + nc] = value<T>(MOM_r[i]);        /// GRM_r
    }
    for (int i = 0; i < nc; ++i) {
        res[0][i + ndof + nc + nc + nc] = value<T>(MOM_l[i]);   /// GRM_l
    }
    return 0;
}


/* In main(), the Recorder is used to save the expression graph of function F.
This expression graph is saved as a MATLAB function named foo.m. From this
function, a c-code can be generated via CasADi and then compiled as a dll. This
dll is then imported in MATLAB as an external function. With this workflow,
CasADi can use algorithmic differentiation to differentiate the function F.
*/
int main() {

    Recorder x[NX];
    Recorder u[NU];
    Recorder p[NP];
    Recorder tau[NR];

    for (int i = 0; i < NX; ++i) x[i] <<= 0;
    for (int i = 0; i < NU; ++i) u[i] <<= 0;
    for (int i = 0; i < NP; ++i) p[i] <<= 0;

    const Recorder* Recorder_arg[n_in] = { x,u,p };
    Recorder* Recorder_res[n_out] = { tau };

    F_generic<Recorder>(Recorder_arg, Recorder_res);

    double res[NR];
    for (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];

    Recorder::stop_recording();

    return 0;

}
