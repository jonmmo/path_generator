function ss = init_search_space(WP_current, WP_next, X)
% Creates a search space between the current- and next waypoint.
%
% Input:
% WP_current - current WP
% WP_next - next WP
% X - size of search space size
%
% ss = create_posibility_set(WP_current, WP_next,x) returns a matrix
% containing (x,y) coordiante of each possibility.
%
% Magnus Knaedal 

x = linspace(WP_current(1), WP_next(1), X);
y = linspace(WP_current(2), WP_next(2), X); 
%figure; %plot(x,y, '*');

ss = [x', y'];
end