/*  This code describes the OpenSim model and the skeleton dynamics
    Author: Antoine Falisse
    Contributor: Joris Gillis, Gil Serrancoli, Chris Dembia
*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce_smooth.h>
#include "SimTKcommon/internal/recorder.h"

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
    joint accelerations (controls u), and returns the joint torques as well as
    several variables for use in the optimal control problems. F is templatized
    using type T. F(x,u)->(r).
*/

// Inputs/outputs of function F
/// number of vectors in inputs/outputs of function F
constexpr int n_in = 2;
constexpr int n_out = 1;
/// number of elements in input/output vectors of function F
constexpr int ndof = 31;        // # degrees of freedom (including locked)
constexpr int NX = ndof*2;      // # states
constexpr int NU = ndof;        // # controls
constexpr int NR = ndof+4*3;    // # residual torques + # joint origins + # GRFs

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

    ///////////////////////////////////////////////////////////////////////////
    // This part was automatically generated using buildModelCpp.m with
    // subject1_v2.osim as input.
    // OpenSim: create components
    /// Model
    OpenSim::Model* model;
    /// Bodies
    OpenSim::Body* pelvis;
    OpenSim::Body* femur_r;
    OpenSim::Body* tibia_r;
    OpenSim::Body* talus_r;
    OpenSim::Body* calcn_r;
    OpenSim::Body* toes_r;
    OpenSim::Body* femur_l;
    OpenSim::Body* tibia_l;
    OpenSim::Body* talus_l;
    OpenSim::Body* calcn_l;
    OpenSim::Body* toes_l;
    OpenSim::Body* torso;
    OpenSim::Body* humerus_r;
    OpenSim::Body* ulna_r;
    OpenSim::Body* radius_r;
    OpenSim::Body* hand_r;
    OpenSim::Body* humerus_l;
    OpenSim::Body* ulna_l;
    OpenSim::Body* radius_l;
    OpenSim::Body* hand_l;
    /// Joints
    OpenSim::CustomJoint* ground_pelvis;
    OpenSim::CustomJoint* hip_r;
    OpenSim::CustomJoint* knee_r;
    OpenSim::CustomJoint* ankle_r;
    OpenSim::CustomJoint* subtalar_r;
    OpenSim::WeldJoint* mtp_r;
    OpenSim::CustomJoint* hip_l;
    OpenSim::CustomJoint* knee_l;
    OpenSim::CustomJoint* ankle_l;
    OpenSim::CustomJoint* subtalar_l;
    OpenSim::WeldJoint* mtp_l;
    OpenSim::CustomJoint* back;
    OpenSim::CustomJoint* shoulder_r;
    OpenSim::CustomJoint* elbow_r;
    OpenSim::CustomJoint* radioulnar_r;
    OpenSim::WeldJoint* radius_hand_r;
    OpenSim::CustomJoint* shoulder_l;
    OpenSim::CustomJoint* elbow_l;
    OpenSim::CustomJoint* radioulnar_l;
    OpenSim::WeldJoint* radius_hand_l;
    /// Model
    model = new OpenSim::Model();
    /// Bodies
    pelvis= new OpenSim::Body("pelvis",8.8426, Vec3(-0.068278,0,0), Inertia(0.07418,0.07418,0.040546,0,0,0));
    model->addBody(pelvis);
    femur_r= new OpenSim::Body("femur_r",6.9838, Vec3(0,-0.17047,0), Inertia(0.10109,0.026499,0.1066,0,0,0));
    model->addBody(femur_r);
    tibia_r= new OpenSim::Body("tibia_r",2.7837, Vec3(0,-0.18049,0), Inertia(0.035366,0.0035787,0.035857,0,0,0));
    model->addBody(tibia_r);
    talus_r= new OpenSim::Body("talus_r",0.075084, Vec3(0,0,0), Inertia(0.00062714,0.00062714,0.00062714,0,0,0));
    model->addBody(talus_r);
    calcn_r= new OpenSim::Body("calcn_r",0.93854, Vec3(0.091392,0.027418,0), Inertia(0.000878,0.0024459,0.0025713,0,0,0));
    model->addBody(calcn_r);
    toes_r= new OpenSim::Body("toes_r",0.16263, Vec3(0.031622,0.0054836,-0.015994), Inertia(6.2714e-05,0.00012543,6.2714e-05,0,0,0));
    model->addBody(toes_r);
    femur_l= new OpenSim::Body("femur_l",6.9838, Vec3(0,-0.17047,0), Inertia(0.10109,0.026499,0.1066,0,0,0));
    model->addBody(femur_l);
    tibia_l= new OpenSim::Body("tibia_l",2.7837, Vec3(0,-0.18049,0), Inertia(0.035366,0.0035787,0.035857,0,0,0));
    model->addBody(tibia_l);
    talus_l= new OpenSim::Body("talus_l",0.075084, Vec3(0,0,0), Inertia(0.00062714,0.00062714,0.00062714,0,0,0));
    model->addBody(talus_l);
    calcn_l= new OpenSim::Body("calcn_l",0.93854, Vec3(0.091392,0.027418,0), Inertia(0.000878,0.0024459,0.0025713,0,0,0));
    model->addBody(calcn_l);
    toes_l= new OpenSim::Body("toes_l",0.16263, Vec3(0.031622,0.0054836,0.015994), Inertia(6.2714e-05,0.00012543,6.2714e-05,0,0,0));
    model->addBody(toes_l);
    torso= new OpenSim::Body("torso",25.7061, Vec3(-0.02676,0.30651,0), Inertia(0.98117,0.45135,0.98117,0,0,0));
    model->addBody(torso);
    humerus_r= new OpenSim::Body("humerus_r",1.5261, Vec3(0,-0.16903,0), Inertia(0.0094704,0.003267,0.01063,0,0,0));
    model->addBody(humerus_r);
    ulna_r= new OpenSim::Body("ulna_r",0.45613, Vec3(0,-0.11828,0), Inertia(0.0021417,0.00044685,0.0023232,0,0,0));
    model->addBody(ulna_r);
    radius_r= new OpenSim::Body("radius_r",0.45613, Vec3(0,-0.11828,0), Inertia(0.0021417,0.00044685,0.0023232,0,0,0));
    model->addBody(radius_r);
    hand_r= new OpenSim::Body("hand_r",0.34351, Vec3(0,-0.066824,0), Inertia(0.00064497,0.00039552,0.00096891,0,0,0));
    model->addBody(hand_r);
    humerus_l= new OpenSim::Body("humerus_l",1.5261, Vec3(0,-0.16903,0), Inertia(0.0094704,0.003267,0.01063,0,0,0));
    model->addBody(humerus_l);
    ulna_l= new OpenSim::Body("ulna_l",0.45613, Vec3(0,-0.11828,0), Inertia(0.0021417,0.00044685,0.0023232,0,0,0));
    model->addBody(ulna_l);
    radius_l= new OpenSim::Body("radius_l",0.45613, Vec3(0,-0.11828,0), Inertia(0.0021417,0.00044685,0.0023232,0,0,0));
    model->addBody(radius_l);
    hand_l= new OpenSim::Body("hand_l",0.34351, Vec3(0,-0.066824,0), Inertia(0.00064497,0.00039552,0.00096891,0,0,0));
    model->addBody(hand_l);
    /// Joints
    //ground_pelvis
    SpatialTransform st_ground_pelvis;
    st_ground_pelvis[0].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tilt", 1, 1));
    st_ground_pelvis[0].setFunction(new LinearFunction());
    st_ground_pelvis[0].setAxis(Vec3(0,0,1));
    st_ground_pelvis[1].setCoordinateNames(OpenSim::Array<std::string>("pelvis_list", 1, 1));
    st_ground_pelvis[1].setFunction(new LinearFunction());
    st_ground_pelvis[1].setAxis(Vec3(1,0,0));
    st_ground_pelvis[2].setCoordinateNames(OpenSim::Array<std::string>("pelvis_rotation", 1, 1));
    st_ground_pelvis[2].setFunction(new LinearFunction());
    st_ground_pelvis[2].setAxis(Vec3(0,1,0));
    st_ground_pelvis[3].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tx", 1, 1));
    st_ground_pelvis[3].setFunction(new LinearFunction());
    st_ground_pelvis[3].setAxis(Vec3(1,0,0));
    st_ground_pelvis[4].setCoordinateNames(OpenSim::Array<std::string>("pelvis_ty", 1, 1));
    st_ground_pelvis[4].setFunction(new LinearFunction());
    st_ground_pelvis[4].setAxis(Vec3(0,1,0));
    st_ground_pelvis[5].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tz", 1, 1));
    st_ground_pelvis[5].setFunction(new LinearFunction());
    st_ground_pelvis[5].setAxis(Vec3(0,0,1));
    ground_pelvis = new CustomJoint("ground_pelvis", model->getGround(), Vec3(0,0,0), Vec3(0,0,0), *pelvis, Vec3(0,0,0), Vec3(0,0,0),st_ground_pelvis);
    //hip_r
    SpatialTransform st_hip_r;
    st_hip_r[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_r", 1, 1));
    st_hip_r[0].setFunction(new LinearFunction());
    st_hip_r[0].setAxis(Vec3(0,0,1));
    st_hip_r[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_r", 1, 1));
    st_hip_r[1].setFunction(new LinearFunction());
    st_hip_r[1].setAxis(Vec3(1,0,0));
    st_hip_r[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_r", 1, 1));
    st_hip_r[2].setFunction(new LinearFunction());
    st_hip_r[2].setAxis(Vec3(0,1,0));
    st_hip_r[3].setFunction(new MultiplierFunction(new Constant(0), 0.96574));
    st_hip_r[3].setAxis(Vec3(1,0,0));
    st_hip_r[4].setFunction(new MultiplierFunction(new Constant(0), 0.96574));
    st_hip_r[4].setAxis(Vec3(0,1,0));
    st_hip_r[5].setFunction(new MultiplierFunction(new Constant(0), 0.986));
    st_hip_r[5].setAxis(Vec3(0,0,1));
    hip_r = new CustomJoint("hip_r", *pelvis, Vec3(-0.068278,-0.063835,0.082331), Vec3(0,0,0), *femur_r, Vec3(0,0,0), Vec3(0,0,0),st_hip_r);
    //knee_r
    SpatialTransform st_knee_r;
    st_knee_r[0].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
    st_knee_r[0].setFunction(new LinearFunction());
    st_knee_r[0].setAxis(Vec3(0,0,1));
    st_knee_r[1].setFunction(new Constant(0));
    st_knee_r[1].setAxis(Vec3(0,1,0));
    st_knee_r[2].setFunction(new Constant(0));
    st_knee_r[2].setAxis(Vec3(1,0,0));
    st_knee_r[3].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
    st_knee_r[3].setAxis(Vec3(1,0,0));
    st_knee_r[4].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
    st_knee_r[4].setAxis(Vec3(0,1,0));
    st_knee_r[5].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
    st_knee_r[5].setAxis(Vec3(0,0,1));
    knee_r = new CustomJoint("knee_r", *femur_r, Vec3(-0.0045122,-0.39691,0), Vec3(0,0,0), *tibia_r, Vec3(0,0,0), Vec3(0,0,0),st_knee_r);
    //ankle_r
    SpatialTransform st_ankle_r;
    st_ankle_r[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_r", 1, 1));
    st_ankle_r[0].setFunction(new LinearFunction());
    st_ankle_r[0].setAxis(Vec3(-0.10501,-0.17402,0.97913));
    st_ankle_r[1].setFunction(new Constant(0));
    st_ankle_r[1].setAxis(Vec3(0,1,0));
    st_ankle_r[2].setFunction(new Constant(0));
    st_ankle_r[2].setAxis(Vec3(0.97913,0,0.10501));
    st_ankle_r[3].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
    st_ankle_r[3].setAxis(Vec3(1,0,0));
    st_ankle_r[4].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
    st_ankle_r[4].setAxis(Vec3(0,1,0));
    st_ankle_r[5].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
    st_ankle_r[5].setAxis(Vec3(0,0,1));
    ankle_r = new CustomJoint("ankle_r", *tibia_r, Vec3(0,-0.41569,0), Vec3(0,0,0), *talus_r, Vec3(0,0,0), Vec3(0,0,0),st_ankle_r);
    //subtalar_r
    SpatialTransform st_subtalar_r;
    st_subtalar_r[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_r", 1, 1));
    st_subtalar_r[0].setFunction(new LinearFunction());
    st_subtalar_r[0].setAxis(Vec3(0.78718,0.60475,-0.12095));
    st_subtalar_r[1].setFunction(new Constant(0));
    st_subtalar_r[1].setAxis(Vec3(0,1,0));
    st_subtalar_r[2].setFunction(new Constant(0));
    st_subtalar_r[2].setAxis(Vec3(-0.12095,0,-0.78718));
    st_subtalar_r[3].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
    st_subtalar_r[3].setAxis(Vec3(1,0,0));
    st_subtalar_r[4].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
    st_subtalar_r[4].setAxis(Vec3(0,1,0));
    st_subtalar_r[5].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
    st_subtalar_r[5].setAxis(Vec3(0,0,1));
    subtalar_r = new CustomJoint("subtalar_r", *talus_r, Vec3(-0.044572,-0.038339,0.0072383), Vec3(0,0,0), *calcn_r, Vec3(0,0,0), Vec3(0,0,0),st_subtalar_r);
    //mtp_r
    mtp_r = new WeldJoint("mtp_r", *calcn_r, Vec3(0.16341,-0.0018278,0.00098704), Vec3(0,0,0), *toes_r, Vec3(0,0,0), Vec3(0,0,0));
    //hip_l
    SpatialTransform st_hip_l;
    st_hip_l[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_l", 1, 1));
    st_hip_l[0].setFunction(new LinearFunction());
    st_hip_l[0].setAxis(Vec3(0,0,1));
    st_hip_l[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_l", 1, 1));
    st_hip_l[1].setFunction(new LinearFunction());
    st_hip_l[1].setAxis(Vec3(-1,0,0));
    st_hip_l[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_l", 1, 1));
    st_hip_l[2].setFunction(new LinearFunction());
    st_hip_l[2].setAxis(Vec3(0,-1,0));
    st_hip_l[3].setFunction(new MultiplierFunction(new Constant(0), 0.96574));
    st_hip_l[3].setAxis(Vec3(1,0,0));
    st_hip_l[4].setFunction(new MultiplierFunction(new Constant(0), 0.96574));
    st_hip_l[4].setAxis(Vec3(0,1,0));
    st_hip_l[5].setFunction(new MultiplierFunction(new Constant(0), 0.986));
    st_hip_l[5].setAxis(Vec3(0,0,1));
    hip_l = new CustomJoint("hip_l", *pelvis, Vec3(-0.068278,-0.063835,-0.082331), Vec3(0,0,0), *femur_l, Vec3(0,0,0), Vec3(0,0,0),st_hip_l);
    //knee_l
    SpatialTransform st_knee_l;
    st_knee_l[0].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
    st_knee_l[0].setFunction(new LinearFunction());
    st_knee_l[0].setAxis(Vec3(0,0,1));
    st_knee_l[1].setFunction(new Constant(0));
    st_knee_l[1].setAxis(Vec3(0,1,0));
    st_knee_l[2].setFunction(new Constant(0));
    st_knee_l[2].setAxis(Vec3(1,0,0));
    st_knee_l[3].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
    st_knee_l[3].setAxis(Vec3(1,0,0));
    st_knee_l[4].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
    st_knee_l[4].setAxis(Vec3(0,1,0));
    st_knee_l[5].setFunction(new MultiplierFunction(new Constant(0), 1.0027));
    st_knee_l[5].setAxis(Vec3(0,0,1));
    knee_l = new CustomJoint("knee_l", *femur_l, Vec3(-0.0045122,-0.39691,0), Vec3(0,0,0), *tibia_l, Vec3(0,0,0), Vec3(0,0,0),st_knee_l);
    //ankle_l
    SpatialTransform st_ankle_l;
    st_ankle_l[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_l", 1, 1));
    st_ankle_l[0].setFunction(new LinearFunction());
    st_ankle_l[0].setAxis(Vec3(0.10501,0.17402,0.97913));
    st_ankle_l[1].setFunction(new Constant(0));
    st_ankle_l[1].setAxis(Vec3(0,1,0));
    st_ankle_l[2].setFunction(new Constant(0));
    st_ankle_l[2].setAxis(Vec3(0.97913,0,-0.10501));
    st_ankle_l[3].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
    st_ankle_l[3].setAxis(Vec3(1,0,0));
    st_ankle_l[4].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
    st_ankle_l[4].setAxis(Vec3(0,1,0));
    st_ankle_l[5].setFunction(new MultiplierFunction(new Constant(0), 0.96673));
    st_ankle_l[5].setAxis(Vec3(0,0,1));
    ankle_l = new CustomJoint("ankle_l", *tibia_l, Vec3(0,-0.41569,0), Vec3(0,0,0), *talus_l, Vec3(0,0,0), Vec3(0,0,0),st_ankle_l);
    //subtalar_l
    SpatialTransform st_subtalar_l;
    st_subtalar_l[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_l", 1, 1));
    st_subtalar_l[0].setFunction(new LinearFunction());
    st_subtalar_l[0].setAxis(Vec3(-0.78718,-0.60475,-0.12095));
    st_subtalar_l[1].setFunction(new Constant(0));
    st_subtalar_l[1].setAxis(Vec3(0,1,0));
    st_subtalar_l[2].setFunction(new Constant(0));
    st_subtalar_l[2].setAxis(Vec3(-0.12095,0,0.78718));
    st_subtalar_l[3].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
    st_subtalar_l[3].setAxis(Vec3(1,0,0));
    st_subtalar_l[4].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
    st_subtalar_l[4].setAxis(Vec3(0,1,0));
    st_subtalar_l[5].setFunction(new MultiplierFunction(new Constant(0), 0.91392));
    st_subtalar_l[5].setAxis(Vec3(0,0,1));
    subtalar_l = new CustomJoint("subtalar_l", *talus_l, Vec3(-0.044572,-0.038339,-0.0072383), Vec3(0,0,0), *calcn_l, Vec3(0,0,0), Vec3(0,0,0),st_subtalar_l);
    //mtp_l
    mtp_l = new WeldJoint("mtp_l", *calcn_l, Vec3(0.16341,-0.0018278,-0.00098704), Vec3(0,0,0), *toes_l, Vec3(0,0,0), Vec3(0,0,0));
    //back
    SpatialTransform st_back;
    st_back[0].setCoordinateNames(OpenSim::Array<std::string>("lumbar_extension", 1, 1));
    st_back[0].setFunction(new LinearFunction());
    st_back[0].setAxis(Vec3(0,0,1));
    st_back[1].setCoordinateNames(OpenSim::Array<std::string>("lumbar_bending", 1, 1));
    st_back[1].setFunction(new LinearFunction());
    st_back[1].setAxis(Vec3(1,0,0));
    st_back[2].setCoordinateNames(OpenSim::Array<std::string>("lumbar_rotation", 1, 1));
    st_back[2].setFunction(new LinearFunction());
    st_back[2].setAxis(Vec3(0,1,0));
    st_back[3].setFunction(new MultiplierFunction(new Constant(0), 0.96574));
    st_back[3].setAxis(Vec3(1,0,0));
    st_back[4].setFunction(new MultiplierFunction(new Constant(0), 0.96574));
    st_back[4].setAxis(Vec3(0,1,0));
    st_back[5].setFunction(new MultiplierFunction(new Constant(0), 0.986));
    st_back[5].setAxis(Vec3(0,0,1));
    back = new CustomJoint("back", *pelvis, Vec3(-0.09725,0.078708,0), Vec3(0,0,0), *torso, Vec3(0,0,0), Vec3(0,0,0),st_back);
    //shoulder_r
    SpatialTransform st_shoulder_r;
    st_shoulder_r[0].setCoordinateNames(OpenSim::Array<std::string>("arm_flex_r", 1, 1));
    st_shoulder_r[0].setFunction(new LinearFunction());
    st_shoulder_r[0].setAxis(Vec3(0,0,1));
    st_shoulder_r[1].setCoordinateNames(OpenSim::Array<std::string>("arm_add_r", 1, 1));
    st_shoulder_r[1].setFunction(new LinearFunction());
    st_shoulder_r[1].setAxis(Vec3(1,0,0));
    st_shoulder_r[2].setCoordinateNames(OpenSim::Array<std::string>("arm_rot_r", 1, 1));
    st_shoulder_r[2].setFunction(new LinearFunction());
    st_shoulder_r[2].setAxis(Vec3(0,1,0));
    st_shoulder_r[3].setFunction(new MultiplierFunction(new Constant(0), 0.89201));
    st_shoulder_r[3].setAxis(Vec3(1,0,0));
    st_shoulder_r[4].setFunction(new MultiplierFunction(new Constant(0), 0.95783));
    st_shoulder_r[4].setAxis(Vec3(0,1,0));
    st_shoulder_r[5].setFunction(new MultiplierFunction(new Constant(0), 0.89201));
    st_shoulder_r[5].setAxis(Vec3(0,0,1));
    shoulder_r = new CustomJoint("shoulder_r", *torso, Vec3(0.0028143,0.35583,0.15164), Vec3(0,0,0), *humerus_r, Vec3(0,0,0), Vec3(0,0,0),st_shoulder_r);
    //elbow_r
    SpatialTransform st_elbow_r;
    st_elbow_r[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flex_r", 1, 1));
    st_elbow_r[0].setFunction(new LinearFunction());
    st_elbow_r[0].setAxis(Vec3(0.22605,0.022269,0.97386));
    st_elbow_r[1].setFunction(new Constant(0));
    st_elbow_r[1].setAxis(Vec3(0,1,0));
    st_elbow_r[2].setFunction(new Constant(0));
    st_elbow_r[2].setAxis(Vec3(0.97386,0,-0.22605));
    st_elbow_r[3].setFunction(new MultiplierFunction(new Constant(0), 1.0275));
    st_elbow_r[3].setAxis(Vec3(1,0,0));
    st_elbow_r[4].setFunction(new MultiplierFunction(new Constant(0), 1.0275));
    st_elbow_r[4].setAxis(Vec3(0,1,0));
    st_elbow_r[5].setFunction(new MultiplierFunction(new Constant(0), 1.0275));
    st_elbow_r[5].setAxis(Vec3(0,0,1));
    elbow_r = new CustomJoint("elbow_r", *humerus_r, Vec3(0.013506,-0.29416,-0.0098593), Vec3(0,0,0), *ulna_r, Vec3(0,0,0), Vec3(0,0,0),st_elbow_r);
    //radioulnar_r
    SpatialTransform st_radioulnar_r;
    st_radioulnar_r[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_r", 1, 1));
    st_radioulnar_r[0].setFunction(new LinearFunction());
    st_radioulnar_r[0].setAxis(Vec3(0.056398,0.99841,0.001952));
    st_radioulnar_r[1].setFunction(new Constant(0));
    st_radioulnar_r[1].setAxis(Vec3(0,1,0));
    st_radioulnar_r[2].setFunction(new Constant(0));
    st_radioulnar_r[2].setAxis(Vec3(0.001952,0,-0.056398));
    st_radioulnar_r[3].setFunction(new MultiplierFunction(new Constant(0), 0.98133));
    st_radioulnar_r[3].setAxis(Vec3(1,0,0));
    st_radioulnar_r[4].setFunction(new MultiplierFunction(new Constant(0), 0.98133));
    st_radioulnar_r[4].setAxis(Vec3(0,1,0));
    st_radioulnar_r[5].setFunction(new MultiplierFunction(new Constant(0), 0.98133));
    st_radioulnar_r[5].setAxis(Vec3(0,0,1));
    radioulnar_r = new CustomJoint("radioulnar_r", *ulna_r, Vec3(-0.0066014,-0.012764,0.025596), Vec3(0,0,0), *radius_r, Vec3(0,0,0), Vec3(0,0,0),st_radioulnar_r);
    //radius_hand_r
    radius_hand_r = new WeldJoint("radius_hand_r", *radius_r, Vec3(-0.0086328,-0.23144,0.013356), Vec3(0,0,0), *hand_r, Vec3(0,0,0), Vec3(0,0,0));
    //shoulder_l
    SpatialTransform st_shoulder_l;
    st_shoulder_l[0].setCoordinateNames(OpenSim::Array<std::string>("arm_flex_l", 1, 1));
    st_shoulder_l[0].setFunction(new LinearFunction());
    st_shoulder_l[0].setAxis(Vec3(0,0,1));
    st_shoulder_l[1].setCoordinateNames(OpenSim::Array<std::string>("arm_add_l", 1, 1));
    st_shoulder_l[1].setFunction(new LinearFunction());
    st_shoulder_l[1].setAxis(Vec3(-1,0,0));
    st_shoulder_l[2].setCoordinateNames(OpenSim::Array<std::string>("arm_rot_l", 1, 1));
    st_shoulder_l[2].setFunction(new LinearFunction());
    st_shoulder_l[2].setAxis(Vec3(0,-1,0));
    st_shoulder_l[3].setFunction(new MultiplierFunction(new Constant(0), 0.89201));
    st_shoulder_l[3].setAxis(Vec3(1,0,0));
    st_shoulder_l[4].setFunction(new MultiplierFunction(new Constant(0), 0.95783));
    st_shoulder_l[4].setAxis(Vec3(0,1,0));
    st_shoulder_l[5].setFunction(new MultiplierFunction(new Constant(0), 0.89201));
    st_shoulder_l[5].setAxis(Vec3(0,0,1));
    shoulder_l = new CustomJoint("shoulder_l", *torso, Vec3(0.0028143,0.35583,-0.15164), Vec3(0,0,0), *humerus_l, Vec3(0,0,0), Vec3(0,0,0),st_shoulder_l);
    //elbow_l
    SpatialTransform st_elbow_l;
    st_elbow_l[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flex_l", 1, 1));
    st_elbow_l[0].setFunction(new LinearFunction());
    st_elbow_l[0].setAxis(Vec3(-0.22605,-0.022269,0.97386));
    st_elbow_l[1].setFunction(new Constant(0));
    st_elbow_l[1].setAxis(Vec3(0,1,0));
    st_elbow_l[2].setFunction(new Constant(0));
    st_elbow_l[2].setAxis(Vec3(0.97386,0,0.22605));
    st_elbow_l[3].setFunction(new MultiplierFunction(new Constant(0), 1.0275));
    st_elbow_l[3].setAxis(Vec3(1,0,0));
    st_elbow_l[4].setFunction(new MultiplierFunction(new Constant(0), 1.0275));
    st_elbow_l[4].setAxis(Vec3(0,1,0));
    st_elbow_l[5].setFunction(new MultiplierFunction(new Constant(0), 1.0275));
    st_elbow_l[5].setAxis(Vec3(0,0,1));
    elbow_l = new CustomJoint("elbow_l", *humerus_l, Vec3(0.013506,-0.29416,0.0098593), Vec3(0,0,0), *ulna_l, Vec3(0,0,0), Vec3(0,0,0),st_elbow_l);
    //radioulnar_l
    SpatialTransform st_radioulnar_l;
    st_radioulnar_l[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_l", 1, 1));
    st_radioulnar_l[0].setFunction(new LinearFunction());
    st_radioulnar_l[0].setAxis(Vec3(-0.056398,-0.99841,0.001952));
    st_radioulnar_l[1].setFunction(new Constant(0));
    st_radioulnar_l[1].setAxis(Vec3(0,1,0));
    st_radioulnar_l[2].setFunction(new Constant(0));
    st_radioulnar_l[2].setAxis(Vec3(0.001952,0,0.056398));
    st_radioulnar_l[3].setFunction(new MultiplierFunction(new Constant(0), 0.98133));
    st_radioulnar_l[3].setAxis(Vec3(1,0,0));
    st_radioulnar_l[4].setFunction(new MultiplierFunction(new Constant(0), 0.98133));
    st_radioulnar_l[4].setAxis(Vec3(0,1,0));
    st_radioulnar_l[5].setFunction(new MultiplierFunction(new Constant(0), 0.98133));
    st_radioulnar_l[5].setAxis(Vec3(0,0,1));
    radioulnar_l = new CustomJoint("radioulnar_l", *ulna_l, Vec3(-0.0066014,-0.012764,-0.025596), Vec3(0,0,0), *radius_l, Vec3(0,0,0), Vec3(0,0,0),st_radioulnar_l);
    //radius_hand_l
    radius_hand_l = new WeldJoint("radius_hand_l", *radius_l, Vec3(-0.0086328,-0.23144,-0.013356), Vec3(0,0,0), *hand_l, Vec3(0,0,0), Vec3(0,0,0));
    /// Add joints
    model->addJoint(ground_pelvis);
    model->addJoint(hip_l);
    model->addJoint(hip_r);
    model->addJoint(knee_l);
    model->addJoint(knee_r);
    model->addJoint(ankle_l);
    model->addJoint(ankle_r);
    model->addJoint(subtalar_l);
    model->addJoint(subtalar_r);
    model->addJoint(mtp_l);
    model->addJoint(mtp_r);
    model->addJoint(back);
    model->addJoint(shoulder_l);
    model->addJoint(shoulder_r);
    model->addJoint(elbow_l);
    model->addJoint(elbow_r);
    model->addJoint(radioulnar_l);
    model->addJoint(radioulnar_r);
    model->addJoint(radius_hand_l);
    model->addJoint(radius_hand_r);
    ///////////////////////////////////////////////////////////////////////////

    /// Contact elements
    OpenSim::HuntCrossleyForce_smooth* HC_1_r;
    OpenSim::HuntCrossleyForce_smooth* HC_2_r;
    OpenSim::HuntCrossleyForce_smooth* HC_3_r;
    OpenSim::HuntCrossleyForce_smooth* HC_4_r;
    OpenSim::HuntCrossleyForce_smooth* HC_5_r;
    OpenSim::HuntCrossleyForce_smooth* HC_6_r;
    OpenSim::HuntCrossleyForce_smooth* HC_1_l;
    OpenSim::HuntCrossleyForce_smooth* HC_2_l;
    OpenSim::HuntCrossleyForce_smooth* HC_3_l;
    OpenSim::HuntCrossleyForce_smooth* HC_4_l;
    OpenSim::HuntCrossleyForce_smooth* HC_5_l;
    OpenSim::HuntCrossleyForce_smooth* HC_6_l;
    /// Parameters
    osim_double_adouble radiusSphere = 0.032;
    osim_double_adouble stiffness = 1000000;
    osim_double_adouble dissipation = 2.0;
    osim_double_adouble staticFriction = 0.8;
    osim_double_adouble dynamicFriction = 0.8;
    osim_double_adouble viscousFriction = 0.5;
    osim_double_adouble transitionVelocity = 0.2;
    Vec3 normal = Vec3(0, 1, 0);
    osim_double_adouble offset = 0;
    Vec3 locSphere_1_r(0.00190115788407966, -0.021859, -0.00382630379623308);
    Vec3 locSphere_2_r(0.148386399942063, -0.021859, -0.028713422052654);
    Vec3 locSphere_3_r(0.133001170607051, -0.021859, 0.0516362473449566);
    Vec3 locSphere_4_r(0.06, -0.0214476, -0.0187603084619177);
    Vec3 locSphere_5_r(0.0662346661991635, -0.021859, 0.0263641606741698);
    Vec3 locSphere_6_r(0.045, -0.0214476, 0.0618569567549652);
    Vec3 locSphere_1_l(0.00190115788407966, -0.021859, 0.00382630379623308);
    Vec3 locSphere_2_l(0.148386399942063, -0.021859, 0.028713422052654);
    Vec3 locSphere_3_l(0.133001170607051, -0.021859, -0.0516362473449566);
    Vec3 locSphere_4_l(0.06, -0.0214476, 0.0187603084619177);
    Vec3 locSphere_5_l(0.0662346661991635, -0.021859, -0.0263641606741698);
    Vec3 locSphere_6_l(0.045, -0.0214476, -0.0618569567549652);
    /// Left foot contact shere specifications
    HC_1_l = new HuntCrossleyForce_smooth("sphere_1_l", "calcn_l", locSphere_1_l, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_2_l = new HuntCrossleyForce_smooth("sphere_2_l", "calcn_l", locSphere_2_l, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_3_l = new HuntCrossleyForce_smooth("sphere_3_l", "calcn_l", locSphere_3_l, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_4_l = new HuntCrossleyForce_smooth("sphere_4_l", "toes_l", locSphere_4_l, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_5_l = new HuntCrossleyForce_smooth("sphere_5_l", "calcn_l", locSphere_5_l, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_6_l = new HuntCrossleyForce_smooth("sphere_6_l", "toes_l", locSphere_6_l, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    /// Add left foot contact spheres to model
    model->addComponent(HC_1_l);
    HC_1_l->connectSocket_body_sphere(*calcn_l);
    model->addComponent(HC_2_l);
    HC_2_l->connectSocket_body_sphere(*calcn_l);
    model->addComponent(HC_3_l);
    HC_3_l->connectSocket_body_sphere(*calcn_l);
    model->addComponent(HC_4_l);
    HC_4_l->connectSocket_body_sphere(*toes_l);
    model->addComponent(HC_5_l);
    HC_5_l->connectSocket_body_sphere(*calcn_l);
    model->addComponent(HC_6_l);
    HC_6_l->connectSocket_body_sphere(*toes_l);
    /// Right foot contact shere specifications
    HC_1_r = new HuntCrossleyForce_smooth("sphere_1_r", "calcn_r", locSphere_1_r, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_2_r = new HuntCrossleyForce_smooth("sphere_2_r", "calcn_r", locSphere_2_r, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_3_r = new HuntCrossleyForce_smooth("sphere_3_r", "calcn_r", locSphere_3_r, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_4_r = new HuntCrossleyForce_smooth("sphere_4_r", "toes_r", locSphere_4_r, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_5_r = new HuntCrossleyForce_smooth("sphere_5_r", "calcn_r", locSphere_5_r, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_6_r = new HuntCrossleyForce_smooth("sphere_6_r", "toes_r", locSphere_6_r, radiusSphere,
        stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    /// Add right foot contact spheres to model
    model->addComponent(HC_1_r);
    HC_1_r->connectSocket_body_sphere(*calcn_r);
    model->addComponent(HC_2_r);
    HC_2_r->connectSocket_body_sphere(*calcn_r);
    model->addComponent(HC_3_r);
    HC_3_r->connectSocket_body_sphere(*calcn_r);
    model->addComponent(HC_4_r);
    HC_4_r->connectSocket_body_sphere(*toes_r);
    model->addComponent(HC_5_r);
    HC_5_r->connectSocket_body_sphere(*calcn_r);
    model->addComponent(HC_6_r);
    HC_6_r->connectSocket_body_sphere(*toes_r);

    // Initialize system and state
    SimTK::State* state;
    state = new State(model->initSystem());

    // Read inputs
    std::vector<T> x(arg[0], arg[0] + NX);
    std::vector<T> u(arg[1], arg[1] + NU);

    // States and controls
    T ua[NU]; /// joint accelerations (Qdotdots) - controls
    Vector QsUs(NX); /// joint positions (Qs) and velocities (Us) - states

    // Assign inputs to model variables
    /// States
    for (int i = 0; i < NX; ++i) QsUs[i] = x[i];
    /// Controls
    /// OpenSim and Simbody have different state orders so we need to adjust
    auto indicesOSInSimbody = getIndicesOSInSimbody(*model);
    for (int i = 0; i < NU; ++i) ua[i] = u[indicesOSInSimbody[i]];

    // Set state variables and realize
    model->setStateVariableValues(*state, QsUs);
    model->realizeVelocity(*state);

    // Compute residual forces
    /// appliedMobilityForces (# mobilities)
    Vector appliedMobilityForces(ndof);
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

    /// Add contact forces to appliedBodyForces
    /// Right foot
    Array<osim_double_adouble> Force_values_1_r = HC_1_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_2_r = HC_2_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_3_r = HC_3_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_4_r = HC_4_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_5_r = HC_5_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_6_r = HC_6_r->getRecordValues(*state);
    SpatialVec GRF_1_r;
    GRF_1_r[0] = Vec3(Force_values_1_r[9], Force_values_1_r[10], Force_values_1_r[11]);
    GRF_1_r[1] = Vec3(Force_values_1_r[6], Force_values_1_r[7], Force_values_1_r[8]);
    SpatialVec GRF_2_r;
    GRF_2_r[0] = Vec3(Force_values_2_r[9], Force_values_2_r[10], Force_values_2_r[11]);
    GRF_2_r[1] = Vec3(Force_values_2_r[6], Force_values_2_r[7], Force_values_2_r[8]);
    SpatialVec GRF_3_r;
    GRF_3_r[0] = Vec3(Force_values_3_r[9], Force_values_3_r[10], Force_values_3_r[11]);
    GRF_3_r[1] = Vec3(Force_values_3_r[6], Force_values_3_r[7], Force_values_3_r[8]);
    SpatialVec GRF_4_r;
    GRF_4_r[0] = Vec3(Force_values_4_r[9], Force_values_4_r[10], Force_values_4_r[11]);
    GRF_4_r[1] = Vec3(Force_values_4_r[6], Force_values_4_r[7], Force_values_4_r[8]);
    SpatialVec GRF_5_r;
    GRF_5_r[0] = Vec3(Force_values_5_r[9], Force_values_5_r[10], Force_values_5_r[11]);
    GRF_5_r[1] = Vec3(Force_values_5_r[6], Force_values_5_r[7], Force_values_5_r[8]);
    SpatialVec GRF_6_r;
    GRF_6_r[0] = Vec3(Force_values_6_r[9], Force_values_6_r[10], Force_values_6_r[11]);
    GRF_6_r[1] = Vec3(Force_values_6_r[6], Force_values_6_r[7], Force_values_6_r[8]);
    int ncalcn_r = model->getBodySet().get("calcn_r").getMobilizedBodyIndex();
    int ntoes_r = model->getBodySet().get("toes_r").getMobilizedBodyIndex();
    appliedBodyForces[ncalcn_r] = appliedBodyForces[ncalcn_r] + GRF_1_r + GRF_2_r + GRF_3_r + GRF_5_r;
    appliedBodyForces[ntoes_r] = appliedBodyForces[ntoes_r] + GRF_4_r + GRF_6_r;
    /// Left foot
    Array<osim_double_adouble> Force_values_1_l = HC_1_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_2_l = HC_2_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_3_l = HC_3_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_4_l = HC_4_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_5_l = HC_5_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_6_l = HC_6_l->getRecordValues(*state);
    SpatialVec GRF_1_l;
    GRF_1_l[0] = Vec3(Force_values_1_l[9], Force_values_1_l[10], Force_values_1_l[11]);
    GRF_1_l[1] = Vec3(Force_values_1_l[6], Force_values_1_l[7], Force_values_1_l[8]);
    SpatialVec GRF_2_l;
    GRF_2_l[0] = Vec3(Force_values_2_l[9], Force_values_2_l[10], Force_values_2_l[11]);
    GRF_2_l[1] = Vec3(Force_values_2_l[6], Force_values_2_l[7], Force_values_2_l[8]);
    SpatialVec GRF_3_l;
    GRF_3_l[0] = Vec3(Force_values_3_l[9], Force_values_3_l[10], Force_values_3_l[11]);
    GRF_3_l[1] = Vec3(Force_values_3_l[6], Force_values_3_l[7], Force_values_3_l[8]);
    SpatialVec GRF_4_l;
    GRF_4_l[0] = Vec3(Force_values_4_l[9], Force_values_4_l[10], Force_values_4_l[11]);
    GRF_4_l[1] = Vec3(Force_values_4_l[6], Force_values_4_l[7], Force_values_4_l[8]);
    SpatialVec GRF_5_l;
    GRF_5_l[0] = Vec3(Force_values_5_l[9], Force_values_5_l[10], Force_values_5_l[11]);
    GRF_5_l[1] = Vec3(Force_values_5_l[6], Force_values_5_l[7], Force_values_5_l[8]);
    SpatialVec GRF_6_l;
    GRF_6_l[0] = Vec3(Force_values_6_l[9], Force_values_6_l[10], Force_values_6_l[11]);
    GRF_6_l[1] = Vec3(Force_values_6_l[6], Force_values_6_l[7], Force_values_6_l[8]);
    int ncalcn_l = model->getBodySet().get("calcn_l").getMobilizedBodyIndex();
    int ntoes_l = model->getBodySet().get("toes_l").getMobilizedBodyIndex();
    appliedBodyForces[ncalcn_l] = appliedBodyForces[ncalcn_l] + GRF_1_l + GRF_2_l + GRF_3_l + GRF_5_l;
    appliedBodyForces[ntoes_l] = appliedBodyForces[ntoes_l] + GRF_4_l + GRF_6_l;
    /// knownUdot
    Vector knownUdot(ndof);
    knownUdot.setToZero();
    for (int i = 0; i < ndof; ++i) knownUdot[i] = ua[i];
    /// Calculate residual forces
    Vector residualMobilityForces(ndof);
    residualMobilityForces.setToZero();
    model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,
        appliedMobilityForces, appliedBodyForces, knownUdot,
        residualMobilityForces);

    // Extract several joint origins to set constraints in problem
    Vec3 calcn_or_l  = calcn_l->getPositionInGround(*state);
    Vec3 calcn_or_r  = calcn_r->getPositionInGround(*state);

    // Extract ground reaction forces
    SpatialVec GRF_r = GRF_1_r + GRF_2_r + GRF_3_r + GRF_4_r + GRF_5_r + GRF_6_r;
    SpatialVec GRF_l = GRF_1_l + GRF_2_l + GRF_3_l + GRF_4_l + GRF_5_l + GRF_6_l;

    // Extract results
    int nc = 3;
    /// Residual forces
    /// OpenSim and Simbody have different state orders so we need to adjust
    auto indicesSimbodyInOS = getIndicesSimbodyInOS(*model);
    for (int i = 0; i < NU; ++i) res[0][i] =
            value<T>(residualMobilityForces[indicesSimbodyInOS[i]]);
    /// ground reaction forces
    for (int i = 0; i < nc; ++i) {
        res[0][i + ndof] = value<T>(GRF_r[1][i]);       /// GRF_r
    }
    for (int i = 0; i < nc; ++i) {
        res[0][i + ndof + nc] = value<T>(GRF_l[1][i]);  /// GRF_l
    }
    /// Joint origins
    for (int i = 0; i < nc; ++i) {
        res[0][i + ndof + nc + nc] = value<T>(calcn_or_r[i]);      /// calcn_or_r
    }
    for (int i = 0; i < nc; ++i) {
        res[0][i + ndof + nc + nc + nc] = value<T>(calcn_or_l[i]); /// calcn_or_l
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
    Recorder tau[NR];

    for (int i = 0; i < NX; ++i) x[i] <<= 0;
    for (int i = 0; i < NU; ++i) u[i] <<= 0;

    const Recorder* Recorder_arg[n_in] = { x,u };
    Recorder* Recorder_res[n_out] = { tau };

    F_generic<Recorder>(Recorder_arg, Recorder_res);

    double res[NR];
    for (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];

    Recorder::stop_recording();

    return 0;

}
