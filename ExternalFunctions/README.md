External Functions
==================

When solving the optimal control problems, we use external functions in which
we perform, among other, inverse dynamics using the Simbody/OpenSim C++ API.
The external functions are then compiled as dlls from which we create Function instances 
using CasADi in MATLAB. 

We use custom versions of Simbody and OpenSim to enable the use of algorithmic
differentiation through CasADi. Information about how to install these libraries, compile
the cpps, and generate the dlls can be found on this page: https://github.com/antoinefalisse/opensim-core/tree/AD-recorder

In this folder, we release the source code of the external functions (cpps)
as well as the corresponding dlls. Only the dlls are necessary to run the problems.

We developed this code on Windows but provide dylibs for Mac users. We cannot
guarantee that all will work smooth on Mac. Please report any issues: email me at
antoine.falisse@kuleuven.be or submit an issue on GitHub.