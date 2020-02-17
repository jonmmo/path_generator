% Script for optimization approach.
%
% Requires Optimization Toolbox.
%
% Magnus Knaedal 30.08.2019
clear all; close all; 
%% Parameters
h = 0.001; % stepsize
theta = 0:h:1;
n = 8; % # control points
objective = 1; % Objective function

% Tuning variables
order = 8; % not used now
zeta = 2; % wall distance
k_max = 3; % max curvature
posib = 20; % size of search space

% Define waypoints
WP = [-4 4; 0 4; 4 0; 8 4; 12 8; 20 0; 24 4; 28 4]; % zig-zag 
%WP =[0 0; 2 -2; 4 -2; 6 0; 6 2; 2 6; 2 8; 4 10; 6 10; 8 8; 8 6]; % S-shape
% Initial heading
psi_init = 0;

% Calculate blending functions
P_b = blending_function(n,theta); 

% Initialize variables
[CP_prev,psi_prev, Q, plt_strct, v, colorvec] = init_conditions(psi_init);

% Replanning.
repl_seg1 = 20;
repl_seg2 = 20;
color = 1;
theta_x = 0.5;

% Continous variable s feeding in to controller.
s = 0; % s = theta + (i-1) + omega
j = 0;
omega = 0;
om = 0;



for i = 1:length(WP)-1 % for each path segment
    
    WP_current = WP(i,:);
    WP_next = WP(i+1,:);
    psi_next = atan2(WP_next(1,2) - WP_current(1,2), ...
                     WP_next(1,1) - WP_current(1,1));

    CP = init_cp(WP_current, psi_prev, WP_next, psi_next, i, CP_prev);
    CP_opt = quadratic_programming(CP, n, zeta, psi_next);
    
    
    if i == repl_seg1 || i == repl_seg2 % Relpanning
        CP_opt = replan(theta_x, theta, Bezier, CP_opt);        
        color = color + 1;
        
        % Update s
        om = om + theta_x;
        s = (j-1) + om + theta;
    else % Regular
        j = j + 1;
        s = theta + (j-1) + om;
    end
    
    % Calculate arc length
    q = distance(CP_opt, P_b.dot_B_blending);
    Q = Q + q;
    
    % Calculate Bezier
    Bezier = calculate_bezier(CP_opt, P_b);
    
    % Plotting
    plt_strct = plotting(WP_current, WP_next, CP_opt, zeta, Bezier, i, s, v, plt_strct, colorvec, color);
    v = v + 1;
    
    %%
    CP_prev = CP_opt;
    psi_prev = psi_next;
end

%% Plotting
figure(1);
plot(WP(:,1),WP(:,2),'ok','markersize',10); hold on;
legend([plt_strct.fig1_curve(1),plt_strct.fig1_ctrl_p(1), plt_strct.fig1_walls(1)],'Septic $\boldmath{B}(\theta)$', 'Control polygon', 'Walls','Interpreter','latex');
legend('-DynamicLegend','Location','Best');
xlabel('$x$','Interpreter','latex')
ylabel('$y$','Interpreter','latex')

figure(2);
plot(WP(:,1),WP(:,2),'ok','markersize',10); hold on;
legend([plt_strct.fig2_curve(1), plt_strct.fig2_walls(1)],'Septic $\boldmath{B}(\theta)$', 'Walls','Interpreter','latex');
legend('-DynamicLegend','Location','Best');
xlabel('$x$','Interpreter','latex')
ylabel('$y$','Interpreter','latex')

figure(3);
subplot(3,1,2);
z(1) = yline(k_max,'--b');
ylim([-k_max-0.5, k_max+0.5])
yline(-k_max,'--b');
figure(3);
legend([plt_strct.fig3_sub1(1) z(1)],'Septic $\boldmath{B}(\theta)$','$\kappa_{max}$','Interpreter','latex','Location','Best','AutoUpdate','off');


figure(4);
subplot(3,1,1);
legend([plt_strct.x_d(1), plt_strct.y_d(1)],'$x^{''}(\theta)$','$y^{''}(\theta)$','Interpreter','latex','AutoUpdate','off');
subplot(3,1,2);
legend([plt_strct.x_dd(1), plt_strct.y_dd(1)],'$x^{''''}(\theta)$','$y^{''''}(\theta)$','Interpreter','latex','AutoUpdate','off');
subplot(3,1,3);
legend([plt_strct.x_ddd(1), plt_strct.y_ddd(1)],'$x^{(3)}(\theta)$','$y^{(3)}(\theta)$','Interpreter','latex','AutoUpdate','off');
