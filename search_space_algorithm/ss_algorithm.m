function oc = ss_algorithm(P, P_b, ss, kappa_max, zeta, psi_next, Delta_max)
% Search-space algorithm
%
% Input:
% - P: List of control points [P0; P1; P2; P3; - , - , - , P7]
% - P_b: blending functions
% - ss: List containing the search space
% - kappa_max: Maximum curvature
% - zeta: Corridor width
% - psi_next: next heading
% - Delta_max: Maximum distance between two waypoints
%
% Output:
% - oc: Optimal  placement of control points
%
% Magnus Knaedal 28.12.2019

%%
% Initialize list of feasbile combinations
fc = [];
P0 = P(1,:);
P7 = P(end,:);
Zeta = abs(zeta*[cos(psi_next), sin(psi_next)]);

% 3 control points to be placed; 3 loops.
for i = floor(length(ss)/2):length(ss)-1
    for j = floor(length(ss)/2):length(ss)-1
        for k = floor(length(ss)/2):length(ss)-1
            if( (i < j) && (j < k))
            P4 = ss(i,:);
            P5 = ss(j,:);
            P6 = ss(k,:);
            
            % Corridor constraints
            if corridor_constraints(P4, P5, P6, P7, Zeta)
                % Curvature constraint next segment
                if curvature_constraints_next_segment(P0, P4, P5, P6, P7, Delta_max, kappa_max, P_b)
                    Points = [P(1:4,:); P4; P5; P6; P(end,:)];
                    % Curvature constraint current segment
                    K = curvature(Points, P_b.dot_B_blending, P_b.ddot_B_blending);
                    K_max = max(K);
                    if K_max <= kappa_max
                        % Add to list of feasible combinations
                        fc = cat(1,fc, [P4 P5 P6]);
                    end
                end
            end
            end
        end
    end
end

% Initialize variable for best combination
bc = fc(1,:);
best_distance = inf;
for i = 1:1:length(fc)
    Points = [P(1:4,:);
              fc(i,1:2);
              fc(i,3:4);
              fc(i,5:6);
              P(end,:)];

    dist = distance(Points, P_b.dot_B_blending);
    % If distance is lower than current best; update best.
    if dist <= best_distance
        best_distance = dist;
        bc = fc(i,:);
    end
    
end

% optimal combination
oc = [P(1:4,:);
      bc(1,1:2);
      bc(1,3:4);
      bc(1,5:6);
      P(end,:)];
end