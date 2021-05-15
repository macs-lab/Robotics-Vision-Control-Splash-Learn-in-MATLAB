% UW Mechanical Engineering
% Robotics, Visiton, and Mechatronics for Manufacturing, Sp 2021
% HW2 / Lab1
% Warning: This piece of code is not complete. Fill the blanks and add your
% own code to answer the raised questions in the HW.
% Please install Robotics Toolbox first: https://petercorke.com/toolboxes/robotics-toolbox/
%% Load puma 560 model
mdl_puma560

%% Visualize poses
q1 = [0, 0, 0, 0, 0, 0];         % zero angles
q2 = [0, pi/2, -pi/2, 0, 0, 0];  % ready pose, arm up
q3 = [0, 0, -pi/2, 0, 0, 0];     % stretch position
% set azimuth and elevation angle
ae_angle = [138, 30];
%p560.plot3d(q1, 'view', ae_angle)
%clf
%p560.plot3d(q2, 'view', ae_angle)
clf
p560.plot3d(q3, 'view', ae_angle)

%%
% sphere pose
sphere_pose = 
% start robot pose
start_joint_pos = 
% tool center pose
start_TCP_pose = 
% construct grasping pose s2e: The following generates the second matrix in parb b
% of problem 1. I will give you this end result, and ask you to develop
% intuitions.
s2e_translate = [0,0,0.3];
s2e_quaternion = UnitQuaternion(0, [sqrt(2)/2, -sqrt(2)/2, 0]);
s2e_pose = transl(s2e_translate)*(s2e_quaternion.T);
% understanding the grasping pose
% obtain the corresponding rotation matrix
R_grasping = 
% obtain the Euler angle: angles of the z y z rotation sequence
gammar = 
% Obtain the overall rotation axis and angle
[theta_grasp,w_hat_grasp] = 

% compute required end-effector pose for grasping
end_TCP_pose = 
% obtain the joint angles and visulize required end-effector pose
end_joint_pos = 

clf

plot_sphere(sphere_pose(1:3,4), 0.04, 'y');
p560.plot3d(end_joint_pos, 'view', [100,30])

%% joint space trajectory
joint_traj = 
clf
plot_sphere(sphere_pose(1:3,4), 0.04, 'y');
p560.plot3d(joint_traj, 'view', ae_angle)

%% Cartesian trajecotry
% note: use trnorm() to normalize the homogeneous transformation matrix if 
% you see errors due to numerical precision.
cartesian_traj = 
joint_traj2 = p560.ikine6s(cartesian_traj, 'ru');
clf
plot_sphere(sphere_pose(1:3,4), 0.04, 'y');
p560.plot3d(joint_traj, 'view', ae_angle)

%% plot joint trajectory
% TODO: add label for x and y axis
figure,
% joint 1
subplot(3,1,1)
plot(joint_traj(:,1))
hold on
plot(joint_traj2(:,1))
legend('joint traj', 'cart traj')

% joint 2
subplot(3,1,2)
plot(joint_traj(:,2))
hold on
plot(joint_traj2(:,2))
legend('joint traj', 'cart traj')

% joint 3
subplot(3,1,3)
plot(joint_traj(:,3))
hold on
plot(joint_traj2(:,3))
legend('joint traj', 'cart traj')

%% plot end-effector position trajectory
% calculate end-effector position for joint-based trajectory
% assume 50 points in the generated trajectory
traj1_TCP_pose = SE3(repmat(eye(4),1,1,50));
for i = 1:50
    traj1_TCP_pose(i) = p560.fkine(joint_traj(i,:));
end
T1 = traj1_TCP_pose.T;      % homogeneous transformation matrix for 
                            % joint-based trajectory
T2 = cartesian_traj.T;      % homogeneous transformation matrix for 
                            % Cartesian trajectory

% TODO: add label for x and y axis
figure,
% translational x axis
subplot(3,1,1)
plot(squeeze(T1(1,4,:)))
hold on
plot(squeeze(T2(1,4,:)))
legend('joint traj', 'cart traj')

% translational y axis
subplot(3,1,2)
plot(squeeze(T1(2,4,:)))
hold on
plot(squeeze(T2(2,4,:)))
legend('joint traj', 'cart traj')

% translational z axis
subplot(3,1,3)
plot(squeeze(T1(3,4,:)))
hold on
plot(squeeze(T2(3,4,:)))
legend('joint traj', 'cart traj')