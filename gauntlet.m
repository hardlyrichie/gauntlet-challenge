% Get lidar scan datapoints
[x,y] = scan();

% Gradient descent constants
delta = 1.1; 
lambda = .3;

% Use RANSAC and calculate lines, build potential field, return gradient 
g = get_gradient(x,y,delta,lambda);

% Set up ROS
sub_bump = rossubscriber('/bump');
pub = rospublisher('/raw_vel'); 
msg = rosmessage(pub);

% Initial Neato position
syms a b
r = [0 0];
grad = [double(vpa(subs(g(1),[a,b],{r(1), r(2)}))) double(vpa(subs(g(2),[a,b],{r(1), r(2)})))];
heading = pi / 2;

rotateNEATO(heading);

% Vector that saves theoretical positions
r_theo = r;

while 1
    % Wait for the next bump message
    bumpMessage = receive(sub_bump);
    % Check if any of the bump sensors are set to 1 (meaning triggered)
    if any(bumpMessage.Data)
        size(bumpMessage.Data);
        msg.Data = [0.0, 0.0];
        send(pub, msg);
        break;
    end
    
    % Calculate new position using gradient
    r_prev = r;
    r = r + lambda .* grad;
    lambda = delta * lambda;
    
    % Find new heading using atan2
    new_heading = atan2(grad(2),grad(1));
    % Make all headings positive
    if(new_heading < 0)
        new_heading = new_heading + 2*pi;
    end
    
    grad = [double(vpa(subs(g(1),[a,b],{r(1), r(2)}))) double(vpa(subs(g(2),[a,b],{r(1), r(2)})))];
    
    % Rotate NEATO to new heading
    heading = rotateNeato(new_heading,heading);
    % Go forward the distance from delta_r
    distance = vecnorm(r - r_prev);
    driveNeato(distance);
    
    r_theo = [r_theo; r];
    pause(.1)
end

save('theoretical_path', 'r_theo')

% Rotate from current heading to new_heading
% Returns new_heading
function out = rotateNeato(new_heading,heading)
    delta_angle = new_heading - heading;
    rotateNEATO(delta_angle);
    out = new_heading;
end

% Drive forward distance meters at 0.3 m/s
function out = driveNeato(distance)
    driveforward(distance,0.3)
end