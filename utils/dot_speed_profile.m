function v_d = dot_speed_profile(u_d, dt_u_d, dot_B, ddot_B)
% Calculates the speed profile derivative wrt theta
%
% Input:
% - P = control points
% - dot_B, ddot_B: blending functions
%
% Magnus Knaedal 28.04.2020
%%

v_d = zeros(1, length(dot_B));

for i = 1:length(dot_B)
    
    d_x = dot_B(i,1);
    dd_x =  ddot_B(i,1);

    d_y = dot_B(i,2);
    dd_y =  ddot_B(i,2);
    
    v_d(i) = dt_u_d/ norm(dot_B(i,:)) - u_d * (d_x * dd_x + d_y*dd_y) / (d_x^2 + d_y^2)^(3/2);
    if isnan(v_d(i))
        K(i) = 0;
    end

end

end