External Functions

When solving the optimal control problems, we use external functions in which
we perform, among other, inverse dynamics using the Simbody/OpenSim C++ API.
The external functions are then compiled as dlls from which we create Function instances 
using CasADi in MATLAB. 

We use custom versions of Simbody and OpenSim to enable the use of algorithmic
differentiation through CasADi. These custom versions will be made available
upon publication (details will be described at https://simtk.org/projects/3dpredictsim). 

In this folder, we release the source code of the external functions (cpps)
as well as the corresponding dlls. Only the dlls are necessary to run the problems.