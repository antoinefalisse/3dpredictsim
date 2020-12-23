External Functions
==================

When solving the optimal control problems, we use external functions in which
we perform, among other, inverse dynamics using the Simbody/OpenSim C++ API.
The external functions are then compiled as dlls from which we create Function instances 
using CasADi in MATLAB. 

We use custom versions of Simbody and OpenSim to enable the use of algorithmic
differentiation through CasADi. Information about how to install these libraries, compile
the cpps, and generate the dlls can be found on this page: https://github.com/antoinefalisse/opensim-core/tree/AD-recorder

In this folder, we share the dlls of the external functions. The corresponding .cpp files can be found in this folder:
https://github.com/antoinefalisse/opensim-core/tree/AD-recorder/OpenSim/External_Functions

We developed this code on Windows. To get the dylibs (Mac users), follow the instructions on this page:
https://github.com/antoinefalisse/opensim-core/tree/AD-recorder. Please report any issues: email me at
afalisse@stanford.edu or submit an issue on GitHub.

From v2.0:

Our framework currently does not allow loading a model (i.e., osim file) through the C++ API.
As a result, the user has to build the model programmatically. This is tedious and prone to errors.
To facilitate this process, we wrote a MATLAB script (buildModelCpp.m) that loads the osim file and outputs
a txt file with part of the C++ code required to build the model programmatically. This feature is still under
development and does not account for all features of OpenSim. For instance, locked joints in your osim will not be locked
in your C++ code, so you should carefully check the output and correct if necessary. In addition, only bodies and joints
will be considered so you should add manually other components (e.g., contact models). Please let us know for any bugs you
find so that we keep on updating this feature.
 