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