% Script for search-space approach.
%
%
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
zeta = 0.5; % wall distance
k_max = 20; % max curvature
posib = 20; % size of search space
Delta_max = sqrt(128); % maximum distance between two waypoints

% Define waypoints
WP = [-4 4; 0 4; 4 0; 8 4; 12 8; 20 0; 24 4; 28 4]; % zig-zag 
WP =[0 0; 2 -2; 4 -2; 6 0; 6 2; 2 6; 2 8; 4 10; 6 10; 8 8; 8 6]; % S-shape
% Initial heading
psi_init = 0;

% Calculate blending functions
P_b = blending_function(n,theta); 

% Initialize variables
[CP_prev,psi_prev, Q, c, v, colorvec] = init_conditions(psi_init);

repl_seg1 = 20;
repl_seg2 = 20;
color = 1;
theta_x = 0.5;

for i = 1:length(WP)-1 % for each path segment
    
    WP_current = WP(i,:);
    WP_next = WP(i+1,:);
    psi_next = atan2(WP_next(1,2) - WP_current(1,2), ...
                     WP_next(1,1) - WP_current(1,1));

    CP = init_cp(WP_current, psi_prev, WP_next, psi_next, i, CP_prev);
    
    
    %search_space = init_search_space(WP_current, WP_next, posib);
    
    CP_opt = quadratic_programming(CP, n, zeta, psi_next);
    
    if i == repl_seg1 || i == repl_seg2
        CP_opt = replan(theta_x, theta, Bezier, CP_opt);
        
        color = color +1;
    end
    
    
    q = distance(CP_opt, P_b.dot_B_blending);
    Q = Q + q;
    
    % Calculate Bezier
    Bezier = calculate_bezier(CP_opt, P_b);
    
    
    %% Plotting
    figure(1); grid on; axis equal;
    ff(c) = plot(Bezier.B_matrix(:,1), Bezier.B_matrix(:,2), 'Color', colorvec{color}, "LineWidth", 1.5); hold on;
    rr(c) = plot(CP_opt(:,1), CP_opt(:,2), 'k.-', 'markersize', 10); hold on;
    % plot walls:
    u = (WP_next - WP_current)/norm(WP_next - WP_current); % unit vector
    
    s = zeta*u; % distance away
    
    rot = [-s(2), s(1)]; % counter clockwise rotation
    r(c) = plot([WP_current(1); WP_next(1)]-rot(1,1),[WP_current(2); WP_next(2)]-rot(1,2),'--b'); hold on; 
           plot([WP_current(1); WP_next(1)]+rot(1,1),[WP_current(2); WP_next(2)]+rot(1,2),'--b'); hold on;

           
    figure(2); grid on; axis equal;
    oo(c) = plot(Bezier.B_matrix(:,1), Bezier.B_matrix(:,2), 'Color', colorvec{color}, "LineWidth", 1.5); hold on;
    % plot walls
    u = (WP_next - WP_current)/norm(WP_next - WP_current); % unit vector    
    s = zeta*u;    % distance away    
    rot = [-s(2), s(1)]; % counter clockwise rotation
    pp(c) = plot([WP_current(1); WP_next(1)]-rot(1,1),[WP_current(2); WP_next(2)]-rot(1,2),'--b'); hold on; 
           plot([WP_current(1); WP_next(1)]+rot(1,1),[WP_current(2); WP_next(2)]+rot(1,2),'--b'); hold on;
           
    % Direction and curvature for each type
    figure(3); grid on;
    subplot(3,1,1); grid on;
    hh(v) = plot(theta+i-1, Bezier.direction, 'Color', 'r', "LineWidth", 1.5); hold on;
    xlabel('$\theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
    ylabel('$[deg]$','Interpreter','latex','FontSize',12)
    title('\textbf{Path direction}','Interpreter','latex','FontSize',12)
    % Curvature
    subplot(3,1,2); grid on;
    jj(v) = plot(theta+i-1, Bezier.K, 'Color', 'r',"LineWidth",1.5); hold on;
    xlabel('$\theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12,'fontweight','bold')
    ylabel('$[1/m]$','Interpreter','latex','FontSize',12)
    title('\textbf{Path curvature}','Interpreter','latex','FontSize',12)
    % Rate of change in curvature
    subplot(3,1,3); grid on;
    gg(v) = plot(theta+i-1, Bezier.dot_K, 'Color', 'r',"LineWidth",1.5); hold on;
    xlabel('$\theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
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
legend([ff(1),rr(1),r(1)],'Septic $\boldmath{B}(\theta)$', 'Control polygon', 'Walls','Interpreter','latex');
legend('-DynamicLegend','Location','Best');
xlabel('$x$','Interpreter','latex')
ylabel('$y$','Interpreter','latex')

figure(2);
plot(WP(:,1),WP(:,2),'ok','markersize',10); hold on;
legend([oo(1),pp(1)],'Septic $\boldmath{B}(\theta)$', 'Walls','Interpreter','latex');
legend('-DynamicLegend','Location','Best');
xlabel('$x$','Interpreter','latex')
ylabel('$y$','Interpreter','latex')

figure(3);
subplot(3,1,2);
z(1) = yline(k_max,'--b');
ylim([-k_max-0.5, k_max+0.5])
yline(-k_max,'--b');
figure(3);
legend([hh(1) z(1)],'Septic $\boldmath{B}(\theta)$','$\kappa_{max}$','Interpreter','latex','AutoUpdate','off');
legend('-DynamicLegend','Location','Best','AutoUpdate','off');
