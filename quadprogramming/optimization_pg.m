% Script for optimization approach.
%
% Requires Optimization Toolbox and export_fig package.
%
% Magnus Knaedal 10.06.2020
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

% Define waypoints
%WP = [-4 4; 0 4; 4 0; 8 4; 12 8; 16 4; 20 0; 24 4; 28 4]; % zig-zag 
%WP =[0 0; 2 -2; 4 -2; 6 0; 6 2; 2 6; 2 8; 4 10; 6 10; 8 8; 8 6]; % S-shape
WP =[0 0; 2 0; 6 -4; 10 -4; 14 0; 14 4; 6 12; 6 16; 10 20; 14 20; 18 16]; % S-shape

WP = [0 0; 5 0; 10 -5; 15 -5; 20 0; 20 5 ;5 20; 5 25; 10 30 ; 15 30; 20 25; 20 20 ; 5 5; 5 0]; % 8-shape
% Initial heading
psi_init = 0;

% Calculate blending functions
P_b = blending_function(n,theta); 

% Initialize variables
[CP_prev,psi_prev, Q, plt_strct, v, colorvec] = init_conditions(psi_init);

% Replanning.
repl_seg1 = -3;
repl_seg2 = -5;
repl_seg3 = -7;
color = 1;
theta_x = 0.4;

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
    
    
    if i == repl_seg1 || i == repl_seg2 || i == repl_seg3
        CP_opt = replan(theta_x, theta, Bezier, CP_opt);        
        color = color + 1;
        
        % Update s
        om = om + theta_x;
        s = theta + (j-1) + om;
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

%% Plotting. Sets labels.
figure(1);
fff = plot(WP(:,1),WP(:,2),'ok','markersize',10); hold on;
legend([plt_strct.fig1_curve(1), plt_strct.fig1_ctrl_p(1), fff, plt_strct.fig1_walls(1)],'Septic $\boldmath{B}_{1 \times 2}(\theta)$', 'Control polygon', 'Waypoints', 'Walls','Interpreter','latex');
legend('-DynamicLegend','Location','Best');
xlabel('East $[m]$','Interpreter','latex','FontSize',12)
ylabel('North $[m]$','Interpreter','latex','FontSize',12)

figure(2);
plot(WP(:,1),WP(:,2),'ok','markersize',10); hold on;
%legend([plt_strct.fig2_curve(1), plt_strct.fig2_walls(1)],'Septic $\boldmath{B}_{1 \times 2}(\theta)$', 'Walls','Interpreter','latex');

legend([plt_strct.fig2_curve(1), plt_strct.fig2_curve(3), plt_strct.fig2_curve(5), plt_strct.fig2_curve(7), plt_strct.fig2_walls(1)], '1st $\boldmath{B}_{1 \times 2}(\theta)$', '2nd $\boldmath{B}_{1 \times 2}(\theta)$', '3th $\boldmath{B}_{1 \times 2}(\theta)$', '4th $\boldmath{B}_{1 \times 2}(\theta)$', 'Walls','Interpreter','latex');
legend('-DynamicLegend','Location','Best');
xlabel('$x$','Interpreter','latex')
ylabel('$y$','Interpreter','latex')

%figure(3);
%subplot(3,1,2);
%z(1) = yline(k_max,'--b');
%ylim([-k_max-0.5, k_max+0.5])
%yline(-k_max,'--b');
%legend([plt_strct.fig3_sub1(1) z(1)],'Septic $\boldmath{B}_{1 \times 2}(\theta)$','$\kappa_{max}$','Interpreter','latex','Location','Best','AutoUpdate','off');
Q

figure(4);
subplot(3,1,1);
legend([plt_strct.x_d(1), plt_strct.y_d(1)],'$x^{\theta}(\theta)$','$y^{\theta}(\theta)$','Interpreter','latex','AutoUpdate','off');
subplot(3,1,2);
legend([plt_strct.x_dd(1), plt_strct.y_dd(1)],'$x^{\theta^2}(\theta)$','$y^{\theta^2}(\theta)$','Interpreter','latex','AutoUpdate','off');
subplot(3,1,3);
legend([plt_strct.x_ddd(1), plt_strct.y_ddd(1)],'$x^{\theta^3}(\theta)$','$y^{\theta^3}(\theta)$','Interpreter','latex','AutoUpdate','off');

figure(1)
set(gcf, 'Color', 'w');
figure(3)
set(gcf, 'Color', 'w');
figure(5)
set(gcf, 'Color', 'w');
export_fig(figure(1), 'xy_opt', '-eps');
export_fig(figure(3), 'der_curv_dcurv_opt', '-eps');
export_fig(figure(5), 'speedprof_opt', '-eps');
