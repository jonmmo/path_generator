function CP = replan(theta_x, theta, Bezier, CP_opt)


idx = find(theta == theta_x);

b_x = [Bezier.B_matrix(idx,1);
       Bezier.dot_B_matrix(idx,1); 
       Bezier.ddot_B_matrix(idx,1);
       Bezier.dddot_B_matrix(idx,1);];
b_y = [Bezier.B_matrix(idx,2);
       Bezier.dot_B_matrix(idx,2); 
       Bezier.ddot_B_matrix(idx,2);
       Bezier.dddot_B_matrix(idx,2);];

A = [1 0 0 0;
    -7 7 0 0;
    42 -84 42 0;
    -210 630 -630 210];

x = A\b_x;
y = A\b_y;


CP = [x ,y;
      CP_opt(5:8,:)];

end