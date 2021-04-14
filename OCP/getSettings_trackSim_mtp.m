% This file contains the settings used for the tracking simulations and 
% is loaded in the different files that are processing and analyzing the
% results

function [settings, settings_trials] = getSettings_trackSim_mtp()

settings = [...
    10,1,1,10,1,50,100,50,4,5,1,1,1,1;
    10,1,1,10,1,50,100,50,4,4,1,1,1,1;  % Tmax is 10
    10,1,1,10,1,50,100,50,4,4,1,1,1,1;  % Tmax is 20
    10,1,1,10,1,50,100,50,4,4,1,1,1,1;  % Tmax is 30
    10,1,1,10,1,50,100,50,4,4,1,1,1,1;  % Tmax is 30 (switch sphere 3 and 5)
    10,1,1,10,1,50,50,50,4,4,1,1,1,1;   % Tmax is 50 (switch sphere 3 and 5) + hard bounds
    10,1,1,10,1,50,50,50,4,4,1,1,1,1;   % Tmax is 100 (switch sphere 3 and 5) + hard bounds
    10,1,1,10,1,50,50,50,4,4,1,1,1,1;   % Tmax is 100 (switch sphere 3 and 5) + hard bounds
    10,1,1,10,1,50,50,50,4,4,1,1,1,1;   % Tmax is 100 (switch sphere 3 and 5) + hard bounds + extend bounds ML sphere 2 (up=0.035)
    10,1,1,10,1,50,50,50,4,4,2,1,1,1;   % Tmax is 100 (remove sphere 3 and 5) + hard bounds + extend bounds ML sphere 2 (up=0.055)
    10,1,1,10,1,50,50,50,4,4,2,1,1,1;   % Tmax is 100 (remove sphere 3 and 5) + hard bounds + extend bounds ML sphere 2 (up=0.055)
    10,1,1,10,1,50,50,50,4,4,2,1,1,1;   % Tmax is 100 (remove sphere 3 and 5) + hard bounds + extend bounds ML sphere 2 (up=0.055)
    10,1,1,10,1,50,50,50,4,6,2,1,1,1;   % Tmax is 100 (remove sphere 3 and 5) + hard bounds + extend bounds ML sphere 2 (up=0.055)
    10,1,1,10,1,50,50,50,4,6,3,1,1,1;   % Tmax is 100 (remove sphere 3 and 5) + hard bounds + extend bounds ML sphere 2 (up=0.055)
    10,1,1,10,1,50,50,50,4,5,2,1,1,1;
    10,1,1,10,1,50,50,50,4,5,2,2,1,1;
    10,1,1,10,1,50,50,50,4,5,2,3,1,1;
    10,1,1,10,1,50,50,50,4,5,2,4,1,1;
    10,1,1,10,1,50,50,50,4,5,2,5,1,1;
    10,1,1,10,1,50,50,50,4,5,2,6,1,1;
    10,1,1,10,1,50,50,50,4,5,3,6,1,1;
    10,1,1,10,1,50,50,50,4,4,3,6,1,1;
    10,1,1,10,1,50,50,50,4,6,4,6,1,1;
    10,1,1,10,1,50,50,50,4,6,5,6,1,1;
    10,1,1,10,1,50,50,50,4,6,6,6,1,1;
    10,1,1,10,1,50,50,50,4,6,6,2,1,1;
    % New cases
    10,1,1,10,1,50,50,50,4,6,6,7,2,1;
    10,1,1,10,1,50,50,50,4,6,6,7,2,0;
    10,1,1,10,1,50,50,50,4,6,7,7,2,0;
    10,10,10,10,1,50,50,50,4,6,7,7,2,0];
settings_trials(1).ww = {'14'};
settings_trials(2).ww = {'14'};
settings_trials(3).ww = {'14'};
settings_trials(4).ww = {'14'};
settings_trials(5).ww = {'14'};
settings_trials(6).ww = {'14'};
settings_trials(7).ww = {'14'};
settings_trials(8).ww = {'14','15'};
settings_trials(9).ww = {'14','15'};
settings_trials(10).ww = {'14'};
settings_trials(11).ww = {'14','15'};
settings_trials(12).ww = {'14','15'};
settings_trials(13).ww = {'14','15'};
settings_trials(14).ww = {'14','15'};
settings_trials(15).ww = {'14','15'};
settings_trials(16).ww = {'14','15'};
settings_trials(17).ww = {'14'};
settings_trials(18).ww = {'14'};
settings_trials(19).ww = {'14'};
settings_trials(20).ww = {'14','15'};
settings_trials(21).ww = {'14','15'};
settings_trials(22).ww = {'14','15'};
settings_trials(23).ww = {'14','15'};
settings_trials(24).ww = {'14','15'};
settings_trials(25).ww = {'14','15'};
settings_trials(26).ww = {'14','15'};
settings_trials(27).ww = {'14','15'};
settings_trials(28).ww = {'14','15'};
settings_trials(29).ww = {'14','15'};
settings_trials(30).ww = {'14','15'};
end
