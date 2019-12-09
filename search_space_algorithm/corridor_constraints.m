function validity = corridor_constraints(P4, P5, P6, P7, Zeta)
% Checks if corridor constraints is valid.
%
% Input: 
% - Control points: P4, P5, P6, P7
% - Zeta: Maximum distance [x,y].

c1 = abs(2*P7-P6 - P7);
c2 = abs(4*P7-4*P6 + P5 - P7);
c3 = abs(8*P7-12*P6+6*P5-P4 - P7);

% initialy false
validity = false;

if c1 <= (abs(Zeta))
    if c2 <= (abs(Zeta))
        if c3 <= (abs(Zeta))
            validity = true;
        end
    end
end

end