function v_d = speed_profile(u_d, dot_B)
% Calculates the speed profile
%
% Input:
% - P = control points
% - dot_B, ddot_B: blending functions
%
% Magnus Knaedal 28.04.2020
%%

v_d = zeros(1, length(dot_B));

for i = 1:length(dot_B)
    v_d(i) = u_d / norm(dot_B(i,:));
    if isnan(v_d(i))
        K(i) = 0;
    end

end

end