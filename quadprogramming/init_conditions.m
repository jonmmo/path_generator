function [CP_prev,psi_prev, Q, s, v, colorvec] = init_conditions(psi_init)
% Initialize variables for path-generation
%
% Input:
% - psi_next: next heading
%
% Output:
% - Initialized parameters to run script
%
% Magnus Knaedal 

CP_prev = zeros(8,2);
Q = 0; % Distance
psi_prev = psi_init;


% Plotting

% Color vector
colorvec = {'r','b','g','c'};
% Counters
v = 1;

% Plot struct
s = struct();
s.fig1_curve = 0;
s.fig1_ctrl_p = 0;
s.fig1_walls = 0;
s.fig2_curve = 0;
s.fig2_walls = 0;
s.fig3_sub1 = 0;
s.fig3_sub2 = 0;
s.fig3_sub3 = 0;
s.x_d = 0;
s.y_d = 0;
s.x_dd = 0;
s.y_dd = 0;
s.x_ddd = 0;
s.y_ddd = 0;
end