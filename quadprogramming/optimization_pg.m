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
[CP_prev,psi_prev, Q, c, v, colorvec] = init_conditions(psi_init);

% Replanning.
repl_seg1 = 3;
repl_seg2 = 5;
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
    
    %% Plotting
    % Walls:
    u = (WP_next - WP_current)/norm(WP_next - WP_current); % unit vector
    z = zeta*u; % distance away
    rot = [-z(2), z(1)]; % counter clockwise rotation
    
    % With ctrl points
    figure(1); grid on; axis equal;
    fig1_curve(c) = plot(Bezier.B_matrix(:,1), Bezier.B_matrix(:,2), 'Color', colorvec{color}, "LineWidth", 1.5); hold on;
    fig1_ctrl_p(c) = plot(CP_opt(:,1), CP_opt(:,2), 'k.-', 'markersize', 10); hold on;
    fig1_walls(c) = plot([WP_current(1); WP_next(1)]-rot(1,1),[WP_current(2); WP_next(2)]-rot(1,2),'--b'); hold on; 
           plot([WP_current(1); WP_next(1)]+rot(1,1),[WP_current(2); WP_next(2)]+rot(1,2),'--b'); hold on;
    
    % Without ctrl points
    figure(2); grid on; axis equal;
    fig2_curve(c) = plot(Bezier.B_matrix(:,1), Bezier.B_matrix(:,2), 'Color', colorvec{color}, "LineWidth", 1.5); hold on;
    fig2_walls(c) = plot([WP_current(1); WP_next(1)]-rot(1,1),[WP_current(2); WP_next(2)]-rot(1,2),'--b'); hold on; 
           plot([WP_current(1); WP_next(1)]+rot(1,1),[WP_current(2); WP_next(2)]+rot(1,2),'--b'); hold on;
           
    % Direction and curvature for each type
    figure(3); grid on;
    subplot(3,1,1); grid on;
    fig3_sub1(v) = plot( s , Bezier.direction, 'Color', colorvec{color}, "LineWidth", 1.5); hold on;
    xlabel('$s =\theta+(i-1)+\omega, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
    ylabel('$[deg]$','Interpreter','latex','FontSize',12)
    title('\textbf{Path direction}','Interpreter','latex','FontSize',12)
    % Curvature
    subplot(3,1,2); grid on;
    fig3_sub2(v) = plot( s , Bezier.K, 'Color', colorvec{color},"LineWidth",1.5); hold on;
    xlabel('$s =\theta+(i-1)+\omega, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12,'fontweight','bold')
    ylabel('$[1/m]$','Interpreter','latex','FontSize',12)
    title('\textbf{Path curvature}','Interpreter','latex','FontSize',12)
    % Rate of change in curvature
    subplot(3,1,3); grid on;
    fig3_sub3(v) = plot( s , Bezier.dot_K, 'Color', colorvec{color},"LineWidth",1.5); hold on;
    xlabel('$s =\theta+(i-1)+\omega, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
    ylabel('$[1/m^2]$','Interpreter','latex','FontSize',12)
    title('\textbf{Rate of change in path curvature}','Interpreter','latex','FontSize',12)
    v = v + 1;
    
    %%
    CP_prev = CP_opt;
    psi_prev = psi_next;
end

%% Plotting
figure(1);
plot(WP(:,1),WP(:,2),'ok','markersize',10); hold on;
legend([fig1_curve(1),fig1_ctrl_p(1),fig1_walls(1)],'Septic $\boldmath{B}(\theta)$', 'Control polygon', 'Walls','Interpreter','latex');
legend('-DynamicLegend','Location','Best');
xlabel('$x$','Interpreter','latex')
ylabel('$y$','Interpreter','latex')

figure(2);
plot(WP(:,1),WP(:,2),'ok','markersize',10); hold on;
legend([fig2_curve(1),fig2_walls(1)],'Septic $\boldmath{B}(\theta)$', 'Walls','Interpreter','latex');
legend('-DynamicLegend','Location','Best');
xlabel('$x$','Interpreter','latex')
ylabel('$y$','Interpreter','latex')

figure(3);
subplot(3,1,2);
z(1) = yline(k_max,'--b');
ylim([-k_max-0.5, k_max+0.5])
yline(-k_max,'--b');
figure(3);
legend([fig3_sub1(1) z(1)],'Septic $\boldmath{B}(\theta)$','$\kappa_{max}$','Interpreter','latex','Location','Best','AutoUpdate','off');
