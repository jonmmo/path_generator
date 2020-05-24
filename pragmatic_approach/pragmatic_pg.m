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

% Tuning variables
zeta = 1.7; % wall distance
k_max = 5; % max curvature
delta_min = 1.5; % Minimum distance from P4 to P7
my = 3; % Scaling factor for distance between P4, P5, P6, and P7

% Define waypoints
%WP = [-4 4; 0 4; 4 0; 8 4; 12 8; 20 0; 24 4; 28 4]; % zig-zag 
%WP =[0 0; 2 -2; 4 -2; 6 0; 6 2; 2 6; 2 8; 4 10; 6 10; 8 8; 8 6]; % S-shape

WP =[0 0; 2 0; 6 -4; 10 -4; 14 0; 14 4; 6 12; 6 16; 10 20; 14 20; 18 16]; % S-shape
psi_init = 0; %initial direction

% Initialize parameters
[CP_prev,psi_current, Q, c, v, colorvec] = init_conditions(psi_init);

% Calculate blending functions
P_b = blending_function(n,theta);
    
for i = 1:length(WP)-1 % for each path segment
    
    WP_current = WP(i,:);
    WP_next = WP(i+1,:);
    psi_next = atan2(WP_next(1,2) - WP_current(1,2), ...
                     WP_next(1,1) - WP_current(1,1));
    
    CP = calculate_cp(WP_current, psi_current, WP_next, psi_next, i, delta_min, my, CP_prev);
    

    Bezier = calculate_bezier(CP,P_b); % Calculate Bezier

    q = distance(CP, P_b.dot_B_blending);
    Q = Q + q;

    %% Plotting
    figure(1); grid on; axis equal;
    rr(c) = plot(CP(:,1), CP(:,2), 'k.-', 'markersize', 10); hold on;
    ff(c) = plot(Bezier.B_matrix(:,1), Bezier.B_matrix(:,2), 'Color', colorvec{1}, "LineWidth", 1.5); hold on;
    % plot walls:
    u = (WP_next - WP_current)/norm(WP_next - WP_current); % unit vector
    
    s = zeta*u; % distance away
    
    rot = [-s(2), s(1)]; % counter clockwise rotation
    r(c) = plot([WP_current(1); WP_next(1)]-rot(1,1),[WP_current(2); WP_next(2)]-rot(1,2),'--b'); hold on; 
           plot([WP_current(1); WP_next(1)]+rot(1,1),[WP_current(2); WP_next(2)]+rot(1,2),'--b'); hold on;

    % Direction and curvature for each type
    figure(3); grid on;
    subplot(3,1,1); grid on;
    hh(v) = plot(theta+i-1, Bezier.direction, 'Color', 'r', "LineWidth", 1.5); hold on;
    ylabel('$\psi(s) \: [deg]$','Interpreter','latex','FontSize',12)
    title('Path Direction','Interpreter','latex','FontSize',12)
    % Curvature
    subplot(3,1,2); grid on;
    jj(v) = plot(theta+i-1, Bezier.K, 'Color', 'r',"LineWidth",1.5); hold on;
    ylabel('$\kappa(s) \: [m^{-1}]$','Interpreter','latex','FontSize',12)
    title('Path Curvature','Interpreter','latex','FontSize',12)
    % Rate of change in curvature
    subplot(3,1,3); grid on;
    gg(v) = plot(theta+i-1, Bezier.dot_K, 'Color', 'r',"LineWidth",1.5); hold on;
    ylabel('$\tau(s) \: [(m/s)^{-1}]$','Interpreter','latex','FontSize',12)
    title('Rate of Change in Path Curvature','Interpreter','latex','FontSize',12)
    
    %xlabel('s = $\theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
    
    % Speed profile:
    
    figure(5); grid on;
    subplot(3,1,1); grid on;
    hh(v) = plot(theta+i-1, Bezier.v_d, 'Color', 'r', "LineWidth", 1.5); hold on;
    ylabel('$v(s,t) [m/s]$','Interpreter','latex','FontSize',12)
    title('The Speed Profile and its Respective Derivatives','Interpreter','latex','FontSize',12)
    
    subplot(3,1,2); grid on;
    jj(v) = plot(theta+i-1, Bezier.dtheta_v, 'Color', 'r',"LineWidth",1.5); hold on;
    ylabel('$v^{s}(s,t) \: [m/s]$','Interpreter','latex','FontSize',12)

    subplot(3,1,3); grid on;
    gg(v) = plot(theta+i-1, Bezier.dt_v, 'Color', 'r',"LineWidth",1.5); hold on;
    ylabel('$v^{t}(s,t) \: [m/s^2]$','Interpreter','latex','FontSize',12)
    
    xlabel('$s = \theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)    
    
    figure(4);
    % first derivative
    subplot(3,1,1); grid on;
    x_d(v) = plot(theta+i-1, Bezier.dot_B_matrix(:,1), 'Color', 'r',"LineWidth",1.5); hold on;
    y_d(v) = plot(theta+i-1, Bezier.dot_B_matrix(:,2), 'Color',  'b',"LineWidth",1.5); hold on;
    xlabel('$\theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
    %ylabel('$[deg]$','Interpreter','latex','FontSize',12)
    title('\textbf{First derivative}','Interpreter','latex','FontSize',12)

    % second derivative
    subplot(3,1,2); grid on;
    x_dd(v) = plot(theta+i-1, Bezier.ddot_B_matrix(:,1), 'Color', 'r',"LineWidth",1.5); hold on;
    y_dd(v) = plot(theta+i-1, Bezier.ddot_B_matrix(:,2), 'Color',  'b',"LineWidth",1.5); hold on;
    xlabel('$\theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
    %ylabel('$[deg]$','Interpreter','latex','FontSize',12)
    title('\textbf{Second derivative}','Interpreter','latex','FontSize',12)        
    % third derivative
    subplot(3,1,3); grid on;
    x_ddd(v) = plot(theta+i-1, Bezier.dddot_B_matrix(:,1), 'Color', 'r',"LineWidth",1.5); hold on;
    y_ddd(v) = plot(theta+i-1, Bezier.dddot_B_matrix(:,2), 'Color',  'b',"LineWidth",1.5); hold on;
    xlabel('s = $\theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
    %ylabel('$[deg]$','Interpreter','latex','FontSize',12)
    title('\textbf{Third derivative}','Interpreter','latex','FontSize',12)       
    v = v + 1;
    %%
    CP_prev = CP;
    psi_current = psi_next;

end

%% Plotting
figure(1);
fff = plot(WP(:,1),WP(:,2),'ko','markersize',10); hold on;
legend([ff(1), rr(1), fff, r(1)],'Septic $\boldmath{B}_{1 \times 2}(\theta)$', 'Control polygon', 'Waypoints',  'Walls','Interpreter','latex');
xlabel('East $[m]$','Interpreter','latex','FontSize',12)
ylabel('North $[m]$','Interpreter','latex','FontSize',12)
legend('-DynamicLegend','Location','Best');

figure(4);
subplot(3,1,1);
legend([x_d(1), y_d(1)],'$x^{''}(\theta)$','$y^{''}(\theta)$','Interpreter','latex','AutoUpdate','off');
subplot(3,1,2);
legend([x_dd(1), y_dd(1)],'$x^{''''}(\theta)$','$y^{''''}(\theta)$','Interpreter','latex','AutoUpdate','off');
subplot(3,1,3);
legend([x_ddd(1), y_ddd(1)],'$x^{(3)}(\theta)$','$y^{(3)}(\theta)$','Interpreter','latex','AutoUpdate','off');

Q

figure(1)
set(gcf, 'Color', 'w');
figure(3)
set(gcf, 'Color', 'w');
figure(5)
set(gcf, 'Color', 'w');
export_fig(figure(1), 'xy_prag', '-eps');
export_fig(figure(3), 'der_curv_dcurv_prag', '-eps');
export_fig(figure(5), 'speedprof_prag', '-eps');
