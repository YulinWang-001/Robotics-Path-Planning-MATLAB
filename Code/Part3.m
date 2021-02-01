%% Step Two
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
    x = data.pos_beat_trajectory(1,point);
    y = data.pos_beat_trajectory(2,point);
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
x = data.pos_beat_trajectory(1,:);
y = data.pos_beat_trajectory(2,:);
substraction_m = abs(((x.^2+y.^2)-l1_m.^2-l2_m.^2)./(2*l1_m.*l2_m))-1;
point = find(substraction_m>0);
% After this, we find out that the first and last three points are out of
% the scope
%% Determine the Start and Final Point
% Choose the REAL first and last point
first = 1+size(point,2)/2; % First = 4
last = size(x,2)-size(point,2)/2;  % Last = 26
%% Initialise Velocity 
v = 0.1;
No = 100; % Number of points interpolated between two adjacent points
Dt = 1/No;
Ds = Dt*v;
count = first;
time = 1;
T = cell(26,1);
Trajectory = zeros(4,4,3);
q = zeros(26,6);
J_total = zeros(3*6,6);
thdot = zeros(3,6);
%% Joint Angles & Homogenous Transformation Matrix and Cartesian Trajectory
V = data.vector;
Angles_temp = data.alpha;
Angle_cell = cell(26,1);
for point = first:last
   Angle_cell{point,1} = angvec2r(Angles_temp(point),V);
   T{point,1} = rt2tr(Angle_cell{point,1},data.pos_beat_trajectory(:,point));
end 
%% Interpolation 
for point = first:last-1
    % Calculate the interpolated trajectory 
    npoints = floor(norm(data.pos_beat_trajectory(:,point+1)...
        -data.pos_beat_trajectory(:,point))./Ds);
    
    % Calculate the interpolated trajectory
    traj_middle = ctraj(T{point,1},T{point+1,1},linspace(0,1,npoints));
%     Trajectory = ctraj(traj_middle(:,:,1:end-1),Trajectory,linspace(0,1,npoints));
    Trajectory = cat(3,Trajectory,traj_middle(:,:,1:end-1));
end 
%% Velocities 
% Calculate the velocity between the adjacent two interpolated points
tr = zeros(299,6);
vel_vec = zeros(299,3);
rot_vel = zeros(299,3);
while count < size(Trajectory,3)
    %Difference in position and rotation between two transformations
    tr_temp = tr2delta(Trajectory(:,:,count),Trajectory(:,:,count+1))';
    tr(count,:) = tr_temp;
    %Calculate unit vector pointing in the direction of next point
    d_temp = (Trajectory(1:3,4,count+1)-Trajectory(1:3,4,count))';
    d_temp = d_temp/norm(d_temp);
    %Calculate velocity vector pointing in that direction
    vel_vec_temp = d_temp*v;
    vel_vec(count,:) = vel_vec_temp;
    %Calculate rotational velocity   
    rot_vel_temp = tr(count,:)./Dt;
    rot_vel_temp = rot_vel_temp(4:6);
    rot_vel(count,:) = rot_vel_temp;
    
    count = count + 1;   
end

total_v = [vel_vec, rot_vel];
%% Set the initial guess values and use ikine to get joint angles
% Set the initial joint values to be zero
q = zeros(26,6);
% Set a initial guess value 
q_guess = [0,-45,-45,180,90,0]/180*pi;
for point = first:size(Trajectory,3)-1
    if point == first
        % Apply the Inverse Kinematics 
        q_start = Rcw.ikine(Trajectory(:,:,point),q_guess);
        q(point,:) = q_start;
    else
        % After deciding the start point, we have to make sure that the
        % velocity does not change 
        J = Rcw.jacob0(q(point-1,:));
        q_temp = q(point-1,:)+((pinv(J)*total_v(point-1,:)').*Dt)';
        % Check with forward kinematics how close we are to desired pose
        T_temp = Rcw.fkine(q_temp);
        error = tr2delta(T_temp, Trajectory(:,:,point));
        % Interate until the error is minimised
        while norm(error(4:6)) > 5e-4
            J = Rcw.jacob0(q(point-1,:));
            q_temp = q(point-1,:) + ((pinv(J)*total_v(point-1,:)').*Dt)';
            T_temp = Rcw.fkine(q_temp);
            error = tr2delta(T_temp,Trajectory(:,:,point));
        end
        q(point,:) = q_temp;
        J_total = [J_total;J];
        % Store rotational velocity (to double check whether it's the same 
        % as in the rot_vel matrix)
        thdot_temp = (q(point,:)-q(point-1,:))/Dt;
        thdot = [thdot;thdot_temp];
    end 
end
%% Plotting
% Plot the trajectory 
%Plots the Puma moving over all points
figure
k = 0;
for i=1:100:size(q,1)-first
        hold on 
        %Plot robot
        Rcw.plot(q(first-1+i,:));
        zlim([-1,1]);
        %Plot trajectory
        x = reshape(Trajectory(1,4,1:i),i,1,1);
        y = reshape(Trajectory(2,4,1:i),i,1,1);
        z = reshape(Trajectory(3,4,1:i),i,1,1);
        hold on;
        plot3(x,y,z,'.b');
        %Draw centre of sphere
        plot3(1,0,0,'or');
        hold off; 
end
%% Plot the effector velocity 
time=0:Dt:size(q,1)*Dt;
figure
plot(time(1:end-1),vel_vec(:,1),'b')
hold on 
plot(time(1:end-1),vel_vec(:,2),'r')
hold on
plot(time(1:end-1),sqrt(vel_vec(:,1).^2+(vel_vec(:,2).^2)),'k', 'LineWidth',1.5)
leg = legend('Vx','Vy','|Vtot|');
xlabel('Time [s]')
ylabel('V [rad/s]')
title('Velocity of end effector');
