import casadi.*

F = external('F','PredSim.dll');
vec1 = -ones(87,1);
res1 = full(F(vec1));

F2 = external('F','PredSim_FD.dll', struct('enable_fd',...
            true,'enable_forward',false,'enable_reverse',false,...
            'enable_jacobian',false,'fd_method','forward'));  
vec2 = -ones(58,1);
vec3 = -ones(29,1);
res2 = full(F2(vec2,vec3));

max(abs(res1-res2))