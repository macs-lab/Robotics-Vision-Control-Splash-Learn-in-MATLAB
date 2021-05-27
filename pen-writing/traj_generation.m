%% given information.

% initial joint angles for the robot.
q_start = [3.36779, 1.1432, 0.2242, -0.3641, 1.5230, 0.1472];

% image coordinates of the projected feature points
% this is generated by simulation (assuming zero error in feature
% extraction step)
mdl_puma560
robot = robotWithPen(p560);
imgPoints = robot.project(robot.robot.fkine(q_start))';

% visulize imgPoints
robot.plotImgPoint(imgPoints')
%imgPoints = [891, 1171, 1204, 915; 683, 676, 529, 524]';

% pose of camera w.r.t TCP
tcp2camera = robot.tcp2camera_pose;

% pose of pen tip w.r.t TCP
tcp2pen = robot.tcp2pentip_pose;

% camera parameter (cameraIntrinsics object)
K = robot.camera_matrix;
param = cameraIntrinsics([K(1,1), K(2,2)], [K(1,3), K(2,3)], robot.img_size);

%% create trajectory.
% your code start from here.
% you should create 'traj' variable that contains the moving trajectory.
% 'traj' should defines the poses of TCP w.r.t the robot base.
% 'traj' should be of type 1xN SE3, where N is the number of waypoints in
% your trajecotry.

%% plot trajectory
robot.plotTrajectory(traj)

%% visualize trajectory
figure,
q_list = robot.robot.ikine6s(traj);
robot.robot.plot3d(q_list)

%% helper funciton
% put any helper functions here.