%% Part One
%% Load data 
clear; clc;
% Load the data values 
data = load('assessed_tutorial_data.mat');
%% Create a DH Table and construct a robot 
a2 = 0.4318;
a3 = 0.0202;
d3 = 0.1244;
d4 = 0.4318;
% The sequence of four parameters is [theta, d, a, alpha]
% The determination of DH table - refer to slide 30 - Forward Kinematics
Lm1 = Link([0,0,0,0], 'modified'); 
Lm2 = Link([0,0,0,-pi/2], 'modified'); 
Lm3 = Link([0,d3,a2,0], 'modified'); 
Lm4 = Link([0,d4,a3,-pi/2], 'modified'); 
Lm5 = Link([0,0,0,pi/2], 'modified'); 
Lm6 = Link([0,0,0,-pi/2], 'modified'); 
Rcw = SerialLink([Lm1, Lm2, Lm3, Lm4, Lm5, Lm6], 'name', 'rm');
% propose the joint limits of the robot
Rcw.qlim(1,:) = [-180, 180]*pi/180;
Rcw.qlim(2,:) = [-180, 180]*pi/180;
Rcw.qlim(3,:) = [-180, 180]*pi/180;
Rcw.qlim(4,:) = [-180, 180]*pi/180;
Rcw.qlim(5,:) = [-180, 180]*pi/180;
Rcw.qlim(6,:) = [-180, 180]*pi/180;
%% Determine the Workspace
% Now we would like to determine the spatial coordinate of the end effector
% which has been given in the dataset

% Determine the workspace of the RM robot
% From the Lecture, it can be seen that the robot arm length L1=a2, and
% L2=d4, hence, define the legnth of robot arm 
l1 = a2;
l2 = d4;
for point = 1:29
    x = data.pos_static_trajectory(1,point);
    y = data.pos_static_trajectory(2,point);
    %  It can be observed from the dataset that 

    if (abs(((x^2+y^2)-l1^2-l2^2)/(2*l1*l2))>1)
        abs(((x^2+y^2)-l1^2-l2^2)/(2*l1*l2));
%         fprintf('No solution exist\n');
% it can be noticed that there are six points out of the workspace
    end 
    
end
%% Find out the points that are out of workspace
% The previous section lets us know that six points are over the scope, in
% this sectionm we will figure out what those points are.
l1_m = l1*ones(1,29);
l2_m = l2*ones(1,29);
x = data.pos_static_trajectory(1,:);
y = data.pos_static_trajectory(2,:);
substraction_m = abs(((x.^2+y.^2)-l1_m.^2-l2_m.^2)./(2*l1_m.*l2_m))-1;
k = find(substraction_m>0);
% After this, we find out that the first and last three points are out of
% the scope
%% Determine the Start and Final Point
% Choose the REAL first and last point
first = 1+size(k,2)/2; % First = 4
last = size(x,2)-size(k,2)/2;  % Last = 26
%% Homogenous Transformation Matrix and Cartesian Trajectory
T = cell(26,1);
for point = first:last
    T{point} = transl(data.pos_static_trajectory(:,point));
    if point == first
        traj = T{point};
    else 
        traj = ctraj(T{point-1}, T{point},10);
    end
end 
%% Set the initial guess values and apply inverse kinematics
% Set the initial joint values to be zero
q = zeros(26,6);
% Set a initial guess value 
q_guess = [0,-45,-45,180,90,0]/180*pi;
for point = first:last
    if point == first
        % Apply the Inverse Kinematics 
        q_start = Rcw.ikine(T{first},q_guess);
        q(4,:) = q_start;
    else
        % After deciding the start point, we can just use the coordinate of the
        % previous point as the guess value of ikine
        q_init_temp=Rcw.ikine(T{point},q_start);
        q_start = q_init_temp;
        q(point,:) = q_start;
    end 
end
%% Plotting
% Plot the trajectory 
%Plots the Puma moving over all points
figure
for i = first:last
    plot3(data.pos_static_trajectory(1,i)...
        ,data.pos_static_trajectory(2,i)...
        ,data.pos_static_trajectory(3,i),'.b');
    zlim([-1,1]);
    hold on
    %Draw centre of sphere
    plot3(1,0,0,'or');
    % Plot the robotic arm trajectory
    Rcw.plot(q(i,:));
end 
figure
%%  Plot the change of joint values during motion
subplot(2,3,1)
plot(1:23,q(first:last,1),'r--')
xlabel('No.positions')
ylabel('Joint1 [rad]')
subplot(2,3,2)
plot(1:23,q(first:last,2),'g--')
xlabel('No.positions')
ylabel('Joint2 [rad]')
subplot(2,3,3)
plot(1:23,q(first:last,3),'b--')
xlabel('No.positions')
ylabel('Joint3 [rad]')
subplot(2,3,4)
plot(1:23,q(first:last,4),'r--')
xlabel('No.positions')
ylabel('Joint4 [rad]')
subplot(2,3,5)
plot(1:23,q(first:last,5),'g--')
xlabel('No.positions')
ylabel('Joint5 [rad]')
subplot(2,3,6)
plot(1:23,q(first:last,6),'b--')
xlabel('No.positions')
ylabel('Joint6 [rad]')


        
        
        
        