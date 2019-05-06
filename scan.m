function [x,y] = scan()
% This function collects lidar data, cleans it up, packages it and returns
% x and y coordinates in cartesian.
sub = rossubscriber('/stable_scan');

% Collect data at the room origin
scan_message = receive(sub);
r = scan_message.Ranges(1:end-1);
theta = [0:359]';

[x,y] = cleanDataClean(theta,r);
end

function [x,y] = cleanDataClean(theta,r)
r_clean = nonzeros(r);
theta_clean = theta;
theta_clean(~r) = [];
[x, y] = pol2cart(deg2rad(theta_clean), r_clean);
end



