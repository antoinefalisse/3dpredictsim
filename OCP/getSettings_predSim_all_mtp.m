% This file contains the settings used for the predictive simulations and 
% is loaded in the different files that are processing and analyzing the
% results

% settings(1): average speed: max 2 digits.
% settings(2): NLP error tolerance: 1*10^(-settings(2)).
% settings(3): number of mesh intervals
% settings(4): weight metabolic energy rate
% settings(5): weight joint accelerations
% settings(6): weight arm excitations
% settings(7): weight passive torques
% settings(8): weight muscle activations
% settings(9): power metabolic energy rate
% settings(10): initial guess identifier: 1 quasi-random, 2 data-informed
% settings(11): contact model identifier: 1 generic, 2 subject-specific
% settings(12): initial guess mode identifier for data-informed guesses
%   1 data from average walking motion, 2 data from average running motion 
%   3 partial solution from motion specified in settings(13)
%   4 full solution from motion specified in settings(13)
%   if 1-3 then settings(10) should be 2
%   if 4 then settings(10) does not matter
% settings(13): initial guess case identifier: row number in "settings" of
% the motion used as initial guess when settings(12)=3 || settings(12)=4
% settings(14): weakness hip muscles, 0 no weakness, 50 weakened by 50%,
% 75 weakened by 75%, 90 weakened by 90%,
% settings(15): maximal contraction velocity identifier
%   0 generic values, 1 values*2
% settings(16): weakness ankle plantarflexors, 0 no weakness, 50 weakened 
% by 50%, 75 weakened by 75%, 90 weakened by 90%,
% settings(17): metabolic energy model identifier: 0 Bhargava et al. (2004)
% 1 Umberger et al. (2003), 2 Umberger (2010), 3 Uchida et al. (2016), 
% 4 Umberger (2010) treating muscle lengthening heat rate as 
% Umberger et al. (2003), 5 Umberger (2010) treating negative mechanical 
% work as Umberger et al. (2003)
% settings(18): co-contraction identifier: 0 lower bound activation = 0.05,
% 1 lower bound activation = 0.1, 2 lower bound activation = 0.15,
% 3 lower bound activation = 0.2
function settings = getSettings_predSim_all_mtp()

settings = [    
    % A. Varying prescribed gait speed
    % Quasi-random initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1;    % 1  
    % Data-informed (walking) initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1;    % 2
    % C. Subject-specific contact model
    % Quasi-random initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 1, 2, 0, 0, 0, 0, 0, 0, 0, 1, 1;    % 3
    % Data-informed (walking) initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 1, 1;    % 4   
    % C. 2nd Subject-specific contact model
    % Quasi-random initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 1, 3, 0, 0, 0, 0, 0, 0, 0, 1, 1;    % 5
    % Data-informed (walking) initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 2, 3, 1, 0, 0, 0, 0, 0, 0, 1, 1;    % 6  
    % D. 3nd Subject-specific contact model
    % Quasi-random initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 1, 4, 0, 0, 0, 0, 0, 0, 0, 1, 1;    % 7
    % Data-informed (walking) initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 2, 4, 1, 0, 0, 0, 0, 0, 0, 1, 1;    % 8 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Added constraint on distance between toes origins
    % D. 3nd Subject-specific contact model
    % Quasi-random initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 1, 4, 0, 0, 0, 0, 0, 0, 0, 1, 1;    % 9
    % Data-informed (walking) initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 2, 4, 1, 0, 0, 0, 0, 0, 0, 1, 1;    % 10 
    % Added constraint on distance between toes origins + adjust mtp polynomials
    % D. 3nd Subject-specific contact model
    % Quasi-random initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 1, 4, 0, 0, 0, 0, 0, 0, 0, 2, 1;    % 11
    % Data-informed (walking) initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 2, 4, 1, 0, 0, 0, 0, 0, 0, 2, 1;    % 12 
    % Adjust tendon stiffness
    % Quasi-random initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 1, 4, 0, 0, 0, 0, 0, 0, 0, 2, 2;    % 13
    % Data-informed (walking) initial guess
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 2, 4, 1, 0, 0, 0, 0, 0, 0, 2, 2;    % 14 
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 2, 4, 1, 0, 0, 0, 0, 0, 0, 2, 3;    % 15 
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 2, 4, 1, 0, 0, 0, 0, 0, 0, 2, 4;    % 16 
    1.33, 4, 50, 500, 50000, 1000000, 1000, 2000, 2, 2, 4, 1, 0, 0, 0, 0, 0, 0, 2, 5;    % 17
];

end
