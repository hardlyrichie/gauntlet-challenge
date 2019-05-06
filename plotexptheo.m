load('experimental_path.mat')
load('theoretical_path.mat')
load('ransac.mat')

%Finding the change in time and change in position to find the velocity
time = dataset(22:end,1);
timestep = diff(dataset(22:end,1));
VLexp = diff(dataset(22:end,2));
VRexp = diff(dataset(22:end,3));
VLacc = (VLexp(:,1))./timestep(:,1);
VRacc = (VRexp(:,1))./timestep(:,1);

Vacc = (VLacc+VRacc)/2;
wacc = (VRacc-VLacc)/0.254;

theta = zeros(length(time),1);
x1 = zeros(length(time),1);
y1 = zeros(length(time),1);

% Calculate experimental path points
for i=2:length(timestep)
   theta(i) = theta(i-1) + wacc(i)*timestep(i);
   x1(i) = x1(i-1) + (Vacc(i)*timestep(i))*cos(theta(i));
   y1(i) = y1(i-1) + (Vacc(i)*timestep(i))*sin(theta(i));
end

hold on

% Plot ransac
for i = 1:length(start_points)
    wall = plot([start_points(i,1) end_points(i,1)], [start_points(i,2) end_points(i,2)], 'r');
end

% Plot experimental
exp = plot(x1,y1,'.');
% Plot theoretical
theo = line(r_theo(:,1),r_theo(:,2),'Color','red');
plot(r_theo(:,1),r_theo(:,2),'.r')

axis equal
hold off
xlabel('x (meters)')
ylabel('y (meters)')
legend([wall, exp, theo], 'Gauntlet walls', 'Experimental path points', 'Theoretical path', 'location', 'northwest')
title('Ransac Lines')