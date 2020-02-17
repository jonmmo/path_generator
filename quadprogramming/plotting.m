function plt_strct = plotting(WP_current, WP_next, CP_opt, zeta, Bezier, i, s, v, plt_strct, colorvec, color)

% Walls:
u = (WP_next - WP_current)/norm(WP_next - WP_current); % unit vector
z = zeta*u; % distance away
rot = [-z(2), z(1)]; % counter clockwise rotation

% With ctrl points
figure(1); grid on; axis equal;
plt_strct.fig1_curve(v) = plot(Bezier.B_matrix(:,1), Bezier.B_matrix(:,2), 'Color', colorvec{color}, "LineWidth", 1.5); hold on;
plt_strct.fig1_ctrl_p(v) = plot(CP_opt(:,1), CP_opt(:,2), 'k.-', 'markersize', 10); hold on;
plt_strct.fig1_walls(v) = plot([WP_current(1); WP_next(1)]-rot(1,1),[WP_current(2); WP_next(2)]-rot(1,2),'--b'); hold on; 
       plot([WP_current(1); WP_next(1)]+rot(1,1),[WP_current(2); WP_next(2)]+rot(1,2),'--b'); hold on;

% Without ctrl points
figure(2); grid on; axis equal;
plt_strct.fig2_curve(v) = plot(Bezier.B_matrix(:,1), Bezier.B_matrix(:,2), 'Color', colorvec{color}, "LineWidth", 1.5); hold on;
plt_strct.fig2_walls(v) = plot([WP_current(1); WP_next(1)]-rot(1,1),[WP_current(2); WP_next(2)]-rot(1,2),'--b'); hold on; 
       plot([WP_current(1); WP_next(1)]+rot(1,1),[WP_current(2); WP_next(2)]+rot(1,2),'--b'); hold on;

% Direction and curvature for each type
figure(3); grid on;
subplot(3,1,1); grid on;
plt_strct.fig3_sub1(v) = plot( s , Bezier.direction, 'Color', colorvec{color}, "LineWidth", 1.5); hold on;
xlabel('$s =\theta+(i-1)+\omega, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
ylabel('$[deg]$','Interpreter','latex','FontSize',12)
title('\textbf{Path direction}','Interpreter','latex','FontSize',12)
% Curvature
subplot(3,1,2); grid on;
plt_strct.fig3_sub2(v) = plot( s , Bezier.K, 'Color', colorvec{color},"LineWidth",1.5); hold on;
xlabel('$s =\theta+(i-1)+\omega, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12,'fontweight','bold')
ylabel('$[m^{-1}]$','Interpreter','latex','FontSize',12)
title('\textbf{Path curvature}','Interpreter','latex','FontSize',12)
% Rate of change in curvature
subplot(3,1,3); grid on;
plt_strct.fig3_sub3(v) = plot( s , Bezier.dot_K, 'Color', colorvec{color},"LineWidth",1.5); hold on;
xlabel('$s =\theta+(i-1)+\omega, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
ylabel('$[(m/s)^{-1}]$','Interpreter','latex','FontSize',12)
title('\textbf{Rate of change in path curvature}','Interpreter','latex','FontSize',12)
% Derivatives
figure(4);
% first derivative
subplot(3,1,1); grid on;
plt_strct.x_d(v) = plot(s, Bezier.dot_B_matrix(:,1), 'Color', colorvec{color} ,"LineWidth",1.5); hold on;
plt_strct.y_d(v) = plot(s, Bezier.dot_B_matrix(:,2), 'Color',  colorvec{color} ,"LineWidth",1.5); hold on;
xlabel('$\theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
%ylabel('$[deg]$','Interpreter','latex','FontSize',12)
title('\textbf{First derivative}','Interpreter','latex','FontSize',12)

% second derivative
subplot(3,1,2); grid on;
plt_strct.x_dd(v) = plot(s, Bezier.ddot_B_matrix(:,1), 'Color', colorvec{color} ,"LineWidth",1.5); hold on;
plt_strct.y_dd(v) = plot(s, Bezier.ddot_B_matrix(:,2), 'Color', colorvec{color} , "LineWidth" ,1.5); hold on;
xlabel('$\theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
%ylabel('$[deg]$','Interpreter','latex','FontSize',12)
title('\textbf{Second derivative}','Interpreter','latex','FontSize',12)        
% third derivative
subplot(3,1,3); grid on;
plt_strct.x_ddd(v) = plot(s, Bezier.dddot_B_matrix(:,1), 'Color', colorvec{color} ,"LineWidth",1.5); hold on;
plt_strct.y_ddd(v) = plot(s, Bezier.dddot_B_matrix(:,2), 'Color', colorvec{color} ,"LineWidth",1.5); hold on;
xlabel('$\theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', 'Interpreter','latex','FontSize',12)
%ylabel('$[deg]$','Interpreter','latex','FontSize',12)
title('\textbf{Third derivative}','Interpreter','latex','FontSize',12)


    
end