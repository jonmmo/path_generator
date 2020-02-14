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

%% Objective function
M = spline_matrix(n);
int_product_a = 16.239;

W = int_product_a * (M' * M);
W_456 = W(5:7, 5:7);

x0 = CP_path(1,1); x1 = CP_path(2,1); x2 = CP_path(3,1); x3 = CP_path(4,1);
x4 = CP_path(5,1); x5 = CP_path(6,1); x6 = CP_path(7,1); x7 = CP_path(8,1); 

f4 = x0*W(5,1) + x1*W(5,2) + x2*W(5,3) + x3*W(5,4) + x7*W(5,8) ...
   + x0*W(1,5) + x1*W(2,5) + x2*W(3,5) + x3*W(4,5) + x7*W(8,5);
f5 = x0*W(6,1) + x1*W(6,2) + x2*W(6,3) + x3*W(6,4) + x7*W(6,8) ...
   + x0*W(1,6) + x1*W(2,6) + x2*W(3,6) + x3*W(4,6) + x7*W(8,6);
f6 = x0*W(7,1) + x1*W(7,2) + x2*W(7,3) + x3*W(7,4) + x7*W(7,8) ...
   + x0*W(1,7) + x1*W(2,7) + x2*W(3,7) + x3*W(4,7) + x7*W(8,7);
f = [f4; f5; f6]; 
%% Constraints

% x4 <= x5 <= x6
A1 = [1 -1 0; 0 1 -1];
b1 = [0; 0];

% Inside corridor
A2 = [0 0 -1; 0 1 -4; -1 6 -12];
b2 = [-x7+zeta; -3*x7+zeta; -7*x7+zeta];

% Bigger than x7. TODO add eztra gap here to keep curvature low
A3 = -A2;
b3 = - ([-x7; -3*x7; -7*x7] + zeta/2);

% x1_i+1 <= x2_i+1 <= x3_i+1
A4 = [0 -1 3; 1 -5 8];
b4 = [2*x7; 4*x7];

A = [A1; A2; A3; A4];
b = [b1; b2; b3; b4];

% All greater than zero. lower bound
lb = (x7/2)*ones(3,1); %zeros(3,1);
ub = x7*ones(3,1);

%%  quadratic programming
options = optimoptions('quadprog','Display','none');
[x,fval,exitflag,output,lambda] = quadprog(W_456,f,A,b,[],[],lb,ub,[], options);

CP_opt_path = [CP_path(1:4,:);
               x(1), CP_path(5,2);
               x(2), CP_path(6,2);
               x(3), CP_path(7,2);
               CP_path(end,:)];

           
for i = 1:length(CP_opt_path)
    CP_opt(i,:) = R * CP_opt_path(i,:)';
    CP_opt(i,:) = CP_opt(i,:) + CP(1,:);
    
end

end