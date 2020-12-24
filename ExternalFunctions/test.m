import casadi.*

F = external('F','PredSim_v2.dll');
vec1 = -ones(93,1);
res1 = full(F(vec1));

F2 = external('F','PredSim_v2_FD.dll', struct('enable_fd',...
            true,'enable_forward',false,'enable_reverse',false,...
            'enable_jacobian',false,'fd_method','forward'));  
vec2 = -ones(62,1);
vec3 = -ones(31,1);
res2 = full(F2(vec2,vec3));

max(abs(res1-res2))