function CP_opt = quadratic_programming(CP, n, zeta, psi_next)
% Input: 
% - CP: initial control points
% - n: degree
% - zeta: wall distance
% 

R = [cos(psi_next), -sin(psi_next);
     sin(psi_next), cos(psi_next) ];


for i = 1:length(CP)
    CP_path(i,:) = CP(i,:) - CP(1,:);
    CP_path(i,:) = R' * CP_path(i,:)';
end

CP_path;

%% Objective function
M = spline_matrix(n);
int_product_a = 16.239;

W = int_product_a * (M' * M)
W_456 = W(5:7, 5:7)

x0 = CP_path(1,1); x1 = CP_path(2,1); x2 = CP_path(3,1); x3 = CP_path(4,1);
x4 = CP_path(1,5); x5 = CP_path(6,1); x6 = CP_path(7,1); x7 = CP_path(8,1); 

f4 = x0*W(5,1) + x1*W(5,2) + x2*W(5,3) + x3*W(5,4) + x7*W(5,8) ...
   + x0*W(1,5) + x1*W(2,5) + x2*W(3,5) + x3*W(4,5) + x7*W(8,5);
f5 = x0*W(6,1) + x1*W(6,2) + x2*W(6,3) + x3*W(6,4) + x7*W(6,8) ...
   + x0*W(1,6) + x1*W(2,6) + x2*W(3,6) + x3*W(4,6) + x7*W(8,6);
f6 = x0*W(7,1) + x1*W(7,2) + x2*W(7,3) + x3*W(7,4) + x7*W(7,8) ...
   + x0*W(1,7) + x1*W(2,7) + x2*W(3,7) + x3*W(4,7) + x7*W(8,7);
f = [f4; f5; f6]; 
%% Constraints

% Inside corridor
A1 = [0 0 -1; 0 1 -4; -1 6 -12];
b1 = [x7+zeta; -3*x7+zeta; -7*x7+zeta];

% Bigger than x7. TODO add eztra gap here to keep curvature low
A2 = -A1;
b2 = -[x7; -3*x7; -7*x7];

% x4 <= x5 <= x5
A3 = [1 -1 0; 0 1 -1];
b3 = [0; 0];

% All greater than zero. lower bound
lb = zeros(3,1);


% quadratic programming

end