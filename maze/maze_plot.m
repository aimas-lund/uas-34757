
% This file contains parameters and calculations needed for running
% MatLab with rotorS ROS package for interfacing with a position controlled
% drone

%%
clc;
close all;
clear all;

maze_1;
start = [0, 0];
end_ = [3, 5];

fig_num = 1;

plot_map(map, fig_num);
plot_start_stop(start, end_, fig_num)

%route = [1 1; 2 1; 3 1; 4 1; 5 1; 6 1];

% Run the algorithm to optain the route
% We have to apply the corresponding transformations (cartesian2matlab)
flip_start = flip(start);
greedy_start = [size(map,1)-flip_start(1), flip_start(2)+1];

flip_end = flip(end_);
greedy_end = [size(map,1)-flip_end(1), flip_end(2)+1];

% This algorithm works considering (1,1) as the first element in the matrix
route = greedy_2d(map, greedy_start, greedy_end);

% We have to apply the corresponding transformations from (matlab2cartesian)
route = flip(route, 2);
route = [route(:, 1), size(map,2)-route(:, 2)+1];

% The plot considers the path as cartesian values not as how matlab
% interprets it.
plot_route(route, fig_num);

%%
clc;
close all;
clear all;

fig_num = 2;

maze_1_3D;

start = [0 0 0];
end_ = [2 8 2];

plot_map(map, fig_num, 0);
plot_start_stop(start, end_, fig_num)

%route = [1 1 1; 2 1 2; 3 1 3; 4 1 3; 5 1 3; 6 1 3];
% Run the algorithm to optain the route
% We have to apply the corresponding transformations (cartesian2matlab)
flip_start = flip(start(1, 1:2));
greedy_start = [size(map, 1)-flip_start(1), flip_start(2)+1, start(1, 3)+1];

flip_end = flip(end_(1, 1:2));
greedy_end = [size(map, 1)-flip_end(1), flip_end(2)+1, end_(1, 3)+1];

route = greedy_3d(map, greedy_start, greedy_end);

% We have to apply the corresponding transformations from (matlab2cartesian)
route_c = flip(route(:, 1:2), 2);
route_c = [route_c(:, 1), size(map,2)-route_c(:, 2)+1, route(:, 3)]

% The plot considers the path as cartesian values not as how matlab
% interprets it.
plot_route(route_c, fig_num)

