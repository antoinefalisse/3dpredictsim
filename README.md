3dpredictsim (3d predictive simulations of walking)
===================================================

This repository contains code and data to generate three-dimensional muscle-driven predictive simulations of human gait as described in: [Falisse A, Serrancoli G, Dembia C, Gillis J, Jonkers J, De Groote F. 2019 Rapid predictive simulations with complex musculoskeletal models suggest that diverse healthy and pathological human gaits can emerge from similar control strategies. Journal of the Royal Society Interface 16: 20190402](http://dx.doi.org/10.1098/rsif.2019.0402).

Here is an example of a predictive simulation of walking, based on a complex musculoskeletal model (29 degrees of freedom, 92 muscles, and 6 contact spheres per foot), generated with our framework (it took only about 20 minutes of CPU time on a single core of a standard laptop computer):

<p align="center">
![Predictive simulation of human walking (doi:10.1098/rsif.2019.0402)](doc/images/opensim_predwalking.gif)
</p>

3dpredictsim contains different folders with data and code needed to reproduce all results and figures from the study. The best way to get started is to run OCP/PredSim_all_v2.m and to explore the code from there (make sure you install CasADi beforehand: [see CasADi website](https://web.casadi.org/)).

Thanks for citing our work in any derived publication. Feel free to reach us for any questions: antoine.falisse@kuleuven.be | antoinefalisse@gmail.com | friedl.degroote@kuleuven.be. This code has been developed on Windows using MATLAB2017b. There is no guarantee that it runs smooth on other platforms. Please let us know if you run into troubles.
