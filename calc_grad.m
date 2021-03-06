function g = calc_grad(start_points,end_points,weights,delta,lambda)
bob = [.9144 1.8288];
f = build_function(start_points,end_points,weights,bob);
g = descent(f,start_points,end_points,delta,lambda,bob);
end

function f = build_function(start_points,end_points,weights,bob)
syms f a b c
f = 0;
vert_tweak = .35;
for i=1:length(start_points)
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
f = f + log10(sqrt((a-bob(1)).^2 + (b-bob(2)).^2))*3;
end

function g = descent(f,start_points,end_points,delta,lambda,bob)
[x,y]=meshgrid(-0.5:.1:1.7,-0.5:.1:2.5);
syms a b
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
% Another method is when gradient goes from getting smaller to suddenly getting bigger
while abs(vecnorm(r - bob)) > .5
    quiver(r(1),r(2),vis_grad(1),vis_grad(2));
    % Calculate new position
    r = r + lambda * grad;
    lambda = delta * lambda;
    grad = [double(vpa(subs(g(1),[a,b],{r(1), r(2)}))) double(vpa(subs(g(2),[a,b],{r(1), r(2)})))];

    vis_grad = grad/norm(grad);
end

hold off
axis equal
end