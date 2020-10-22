OCP (Optimal Control Problems)
==============================

To run the scripts contained in this folder, you should install
CasADi: https://web.casadi.org/.

Note that these scripts have only been tested on Windows using MATLAB2017b.

### OCP (Optimal Control Problems)

1. PredSim_all_v2.m
    - Latest version of the script used to formulate the predictive simulations (except those with the prosthesis).
3. PredSim_all.m
    - Script used to formulate the predictive simulations (except those with the prosthesis). This script was used to generate the results of the [paper](http://dx.doi.org/10.1098/rsif.2019.0402). This script is no longer used in newer developments so that users can fully replicate our results. For a newer version of the code, please use PredSim_all_v2.m.
4. PredSim_prosthesis.m
    - Script used to formulate the predictive simulations with the prosthesis. This script was used to generate the results of the [paper](http://dx.doi.org/10.1098/rsif.2019.0402). This script is no longer used in newer developments so that users can fully replicate our results.
5. TrackSim.m
    - Script used to formulate the tracking simulations. This script was used to generate the results of the [paper](http://dx.doi.org/10.1098/rsif.2019.0402). This script is no longer used in newer developments so that users can fully replicate our results.
7. getSettings_<>.m
    - Functions to get settings for use in the main scripts.
    