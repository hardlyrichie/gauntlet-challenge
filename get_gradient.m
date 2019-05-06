% This script combines calc_ransac and calc_grad for faster runtime as
% there is less passing symbolics between functions
function g = get_gradient(x,y,delta,lambda)
[start_points,end_points,weights] = calc_ransac(x,y);
[x,y]=meshgrid(-0.5:.1:1.7,-0.5:.1:2.5);
syms f a b c
f = 0;
% Weight vertical lines
vert_tweak = .3;
for i = 1:length(start_points)
    % Evaluate indefinte integral
    if abs(weights(i)) > 1 % Closer to vertical line, integrate with respect to y
        s = int(log(sqrt((a-((c-weights(i,2))/weights(i,1))).^2 + (b-c).^2)),c);
        % Perform definite integration using the indefinite integral
        % evaluations by substituting in values for c
        % Make sure start point comes before end point in integration
        if start_points(i,2) < end_points(i,2)
            f = f - (subs(s,c,end_points(i,2)) - subs(s,c,start_points(i,2)))*vert_tweak;
        else
            f = f - (subs(s,c,start_points(i,2)) - subs(s,c,end_points(i,2)))*vert_tweak;
        end
    else % Closer to horizontal line, integrate with respect to x
        s = int(log(sqrt((a-c).^2 + (b-(weights(i,1)*c+weights(i,2))).^2)),c);
        if start_points(i,1) < end_points(i,1)
            f = f - (subs(s,c,end_points(i,1)) - subs(s,c,start_points(i,1)));
        else
            f = f - (subs(s,c,start_points(i,1)) - subs(s,c,end_points(i,1)));
        end
    end
end

% Manually add bob sink
bob = [.9144 1.8288];
f = f + log10(sqrt((a-bob(1)).^2 + (b-bob(2)).^2))*2;

figure;
hold on;
% Plot ransac lines (lines fitted for walls & obstacles) that define potential field
for i = 1:length(start_points)
    plot([start_points(i,1) end_points(i,1)], [start_points(i,2) end_points(i,2)], 'r');
end

% Calculate negative gradient for descent
g = -gradient(f,[a,b]);

% Evaluate symbolic function for every x and y point in meshgrid. Convert
% the symbolic to a double precision
z = double(vpa(subs(f,[a, b],{x, y})));

% Plot contour of potential field
contour(x,y,z,'ShowText','on');
% surf(x,y,z)

% Plot quivors
% gx = subs(g(1),[a,b],{x, y});
% gy = subs(g(2),[a,b],{x, y});
% quiver(x,y,gx,gy,'AutoScaleFactor',3)

% ***** Gradient descent *****
% Initial values
r = [0 0];
grad = [double(vpa(subs(g(1),[a,b],{r(1), r(2)}))) double(vpa(subs(g(2),[a,b],{r(1), r(2)})))];
vis_grad = grad/norm(grad); % Normalize gradient to visualize on plot

% While gradient is greater than some threshold.
while abs(vecnorm(r - bob)) > .5
    quiver(r(1),r(2),vis_grad(1),vis_grad(2));
    % Calculate new position
    r = r+lambda*grad; 
    lambda = delta * lambda;
    grad = [double(vpa(subs(g(1),[a,b],{r(1), r(2)}))) double(vpa(subs(g(2),[a,b],{r(1), r(2)})))];

    vis_grad = grad/norm(grad);
end

hold off
axis equal
xlabel('x (meters)')
ylabel('y (meters)')
title('Theoretical Path of Neato Gradient Descent')
end

function [start_points,end_points,weights] = calc_ransac(x,y)
% This function given x and y coordinates of a lidar scan, determines the
% line segments that best fit the data points using ransac.
% Returns start_points: matrix of starting points for line segments,
% end_points: matrix of ending points for line segments,
% weights: corresponding m and b values for the line segments
start_points = [];
end_points = [];
weights = [];
d = .05;
while length(x) > 1
    [p1, p2, max_inliers, c] = robustLineFit(x, y, d, length(x));

    if p1 ~= p2
        start_points = [start_points; p1'];
        end_points = [end_points; p2'];
        weights = [weights; c];
    end

    % Remove inlier points
    for j = flipud(max_inliers)
        x(j) = [];
        y(j) = [];
    end
end
save('ransac.mat', 'start_points', 'end_points', 'weights');
end

function [p1, p2, max_inliers, c] = robustLineFit(x, y, d, n)
max_inliers = [];
c = [];
p1 = 0;
p2 = 0;
gap_size = .3;
for i = 1:n
    % Define unit vectors t, k, n for random points a and b
    [a, b] = pick2(x,y);
    t = (b - a)/norm(b - a);
    k = [0;0;1];
    n = cross(k, [t;0]);

    inliers_x = [];
    inliers_y = [];
    inliers = [];

    % Determine perpendicular distance and number of inliers for all 
    % other points in data from line given by points a and b
    for j = 1:length(x)    
        j_x = x(j,1);
        j_y = y(j,1);
        r = [j_x;j_y] - a;
        dis = abs(dot([r;0], n));

        if dis < d        
            inliers_x = [inliers_x; x(j)];
            inliers_y = [inliers_y; y(j)];
            inliers = [inliers; j];
        end 
    end

    % Store start point and end point if line given by points a and b
    % result in the greatest number of inliers seen so far
    if length(inliers_x) > length(max_inliers)
        % Define equation of line
        c = get_line(a, b);
        m = abs(c(1,1));
        if m > 1
            [M1, I1] = min(inliers_y);
            [M2, I2] = max(inliers_y);
            p1 = [inliers_x(I1); inliers_y(I1)];
            p2 = [inliers_x(I2); inliers_y(I2)];

            [inliers_y, sorted_idx] = sort(inliers_y);
            inliers_x = inliers_x(sorted_idx);
            inliers = inliers(sorted_idx);

            for j = 2:length(inliers)
                if pdist([inliers_x(j),inliers_y(j);inliers_x(j-1),inliers_y(j-1)]) > gap_size
                   p2 = [inliers_x(j-1); inliers_y(j-1)];
                   inliers = inliers(1:j-1,1);
                   break;
                end
            end
        end
        if m <= 1
            [M1, I1] = min(inliers_x);
            [M2, I2] = max(inliers_x);
            p1 = [inliers_x(I1); inliers_y(I1)];
            p2 = [inliers_x(I2); inliers_y(I2)];

            [inliers_x, sorted_idx] = sort(inliers_x);
            inliers_y = inliers_y(sorted_idx);
            inliers = inliers(sorted_idx);

            for j = 2:length(inliers)
                if pdist([inliers_x(j),inliers_y(j);inliers_x(j-1),inliers_y(j-1)]) > gap_size                     p2 = [inliers_x(j-1); inliers_y(j-1)];
                   inliers = inliers(1:j-1,1);
                   break;
                end
            end
        end          
        max_inliers = inliers;
    end
end 
end

function coefficients = get_line(a, b)
% Return coefficients for the best linear fit between points a and b
coefficients = polyfit([a(1,1), b(1,1)], [a(2,1), b(2,1)], 1);
end

function [a, b] = pick2(x,y)
% Return two random unique points from the list of points given by x
% and y
index = randsample(length(x),2);
a = [x(index(1), 1); y(index(1), 1)];
b = [x(index(2), 1); y(index(2), 1)];
end