function validity = curvature_constraints_next_segment(P0, P4, P5, P6, P7, Delta_max, kappa_max, P_b)
% Check if curvature constraint is valid for next segment.
%
% Input:
% - Control points: P0, P4, P5, P6, P7
% - Delta_max: Maximum distance between two waypoints
% - kappa_max: Maximum curvature
% - P_b: blending functions 


% Check max curvature ok for next segment
% Artificial points:


% Create artifical points representing the "worst-case" scenario that can
% happen, a 90deg turn.

% Subtract P7 to work in origo and simplify calculations.
A0 = P7 - P7;
A1 = 2*P7 - P6 - P7;
A2 = 4*P7 - 4*P6 + P5 - P7;
A3 = 8*P7 -12*P6 + 6*P5-P4 - P7;

% unit vector
u = (P7 - P0)/norm(P7 - P0); 

A4 = (Delta_max-0.3)*u;
A5 = (Delta_max-0.2)*u;
A6 = (Delta_max-0.1)*u;
A7 = Delta_max*u;

% Rotate points counter clockwise to construct 90deg turn
A4 = [-A4(2), A4(1)]; 
A5 = [-A5(2), A5(1)];
A6 = [-A6(2), A6(1)];
A7 = [-A7(2), A7(1)];

P_worst = [A0; A1; A2; A3; A4; A5; A6; A7];

% Calculate curvature
K = curvature(P_worst, P_b.dot_B_blending, P_b.ddot_B_blending);
K_max = max(K);

validity = false;
if K_max <= kappa_max
    validity = true;
end

end