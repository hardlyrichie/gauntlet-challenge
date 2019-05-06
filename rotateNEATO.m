function output = rotateNEATO(angle)
% rotate is a simple function that controls the NEATO to rotate 
% at a designated speed for a set angle

%This line says we are going to publish to the topic `/raw_vel' 
%which are the left and right wheel velocities
pubvel = rospublisher('/raw_vel'); 

%Here we are creating a ROS message
message = rosmessage(pubvel); 



%Set the right and left wheel velocities
speed = 0.1;
if angle > 0
message.Data = [-speed, speed]; 
else
message.Data = [speed,-speed];
end

omega = (message.Data(2) - message.Data(1)) ./ 0.254;

% Send the velocity commands to the NEATO
send(pubvel, message);
%in Matlab tic and toc start and stop a timer. In this program we are making sure we 
%rotate the desired angle by finding the necessary time based on speed
driveTime = tic;
while 1
    if toc(driveTime) > abs(angle/omega) % Here we are saying the if the elapsed time is greater than 
        %angle/omega, we have reached our desired angle and we should stop
        
        message.Data = [0,0]; % Set wheel velocities to zero if we have reached the desires angle
        send(pubvel, message); % Send new wheel velocities
        break %leave this loop once we have reached the stopping time
    end
    pause(.1)
end
%subenc.LatestMessage.Data
end