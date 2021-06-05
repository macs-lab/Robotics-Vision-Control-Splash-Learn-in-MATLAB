% University of Washington
% Mechanical Engineering
% Robotics, Vision, and Mechatronics for Manufacturing, 2021 Sp
% 
% required toolbox:
%   Robotics toolbox from Peter Corke.
%   Computer Vision toolbox from Mathworks (to use pose estimation function).
%   Statistics and Machine Learning Toolbox from Mathworks (to generate
%       Gaussian noise)

%% set camera model
camera.K = [1349.056772980567, 0, 774.9519749992618;
            0, 1351.558896220857, 557.5626838601945;
            0, 0, 1];
camera.img_size = [1600, 1200];
camera_param = cameraIntrinsics([camera.K(1,1), camera.K(2,2)],...
                                [camera.K(1,3), camera.K(2,3)], ...
                                camera.img_size);

%% environment setup
% featuer points' coordinates w.r.t the board frame
featurePositions = [-0.1, -0.1, 0.1, 0.1; 
                    -0.1, 0.1,  0.1, -0.1; 
                    0,    0,    0,   0];
         
% the desired pose of board w.r.t camera (4x4 matrix)
goal_camera2board_pose = transl(0,0,0.3)*trotx(pi);

% camera's initial pose w.r.t the board frame (4x4 matrix)
% don't use this vairable in your algorithm
% it is assumed unknown
init_board2camera_pose = transl(-0.5,-0.4,2.5)*trotz(1.6)*trotx(3.1)*troty(0.5);

%% simulation
step_size = 0.005;          % simulation step size in seconds
sim_time = 5;               % simulation duration in seconds
N_steps = sim_time/step_size;

% initialize the state object
% the state object describes all state variables at a time
state_obj = state(init_board2camera_pose, camera, featurePositions);

% we create a cell array to save state object at each time step.
data = cell([N_steps,1]);

% simulation start.
k = 0;
for t = step_size:step_size:sim_time
    k = k + 1;
    % Step 1: compute (by simulation) the image coordinates of projected 
    % feature points
    [state_obj, imgPoints] = state_obj.project();
    
    % Step 2: visual servo step to compute desired camera velocity.
    
    %===========================
    % Your code starts from here
    % You should obtain desired camera velocity (w.r.t to camera frame)
    % cam_vel (6x1 vector) based on the obtained feature points in the image.

    % ---- PBVS without pose estimation noises -----
    % A possible PBVS_step function is written at the end of this file to compute the camera velocity vector under a PBVS algorithm.
    lambda = 1;
    cam_vel = PBVS_step(imgPoints, camera,...
                        featurePositions, goal_camera2board_pose, lambda, 1);

    % ---- IBVS (using the goal depth) -----
    % This is an example code structure. You will need to write your own IBVS_step function.
    %cam_vel = IBVS_step(...)
    % test the case with different Z values

    % ---- IBVS (using the true depth) -----
    %Z = state_obj.getFeatureDepth();
    %cam_vel = IBVS_step(...)

    % ---- PBVS and IBVS with noises in pose estimation -----
    %

    % your code ends here.
    %====================
    
    % Step 3: update camera position. We assume a perfect motion
    % controller. That is, the actual camera velocity equals to the desired
    % camera velocity given by the visual servo step. We assume that the 
    % camera is moving with a constant speed during the step_size time.
    state_obj = state_obj.step(cam_vel, step_size);
    
    % save current state. 
    data{k} = state_obj;
end
% simulation end.

%% analysis and visulize data

%% % Plot image plane trajectory % %
figure,
hold on
num_data = size(data,1);
for i = 1:num_data
    imgPoints = data{i}.imagePoints;
    plot(imgPoints(1,:),imgPoints(2,:),'.b');
    plot(mean(imgPoints(1,:)), mean(imgPoints(2,:)), '.r');
    axis([0 1600 0 1200]);
end
title('Feature trajectory in the image plane')
% Red line is the trajectory of the feature center.

%% % Plot camera center trajectory % %
camera_central.x = zeros(num_data,1);
camera_central.y = zeros(num_data,1);
camera_central.z = zeros(num_data,1);
camera_central.time = zeros(num_data,1);
for i = 1:num_data
    cameraPose = data{i}.cameraPose;
    camera_central.x(i) = cameraPose(1,4);
    camera_central.y(i) = cameraPose(2,4);
    camera_central.z(i) = cameraPose(3,4);
    camera_central.time(i) = data{i}.time;
end
figure,
hold on
plot3(camera_central.x, camera_central.y, camera_central.z);
hold on
plot3(featurePositions(1,:), featurePositions(2,:), featurePositions(3,:), 'or');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-1 1 -1 1 0 4])
title('camera central trajectory');
view(27,37)
axis equal
xlim([-1 1])
ylim([-1 1])
zlim([-0.5 4])

%% plot pose error vs time
rotation_error = zeros(N_steps, 1);
translation_error = zeros(N_steps,1);
time = zeros(N_steps,1);
for i = 1:num_data
    cam_pose = data{i}.cameraPose;
    error_pose = goal_camera2board_pose * cam_pose;
    
    translation_error(i) = norm(tform2trvec(error_pose));
    angaxis = tform2axang(error_pose);
    rotation_error(i) = abs(angaxis(4));
    time(i) = data{i}.time;
end
figure,
hold on
yyaxis left
plot(time, rotation_error*180/pi);
set(gca, 'YScale', 'log')
ylabel('Rotational error (degree)')
yyaxis right
plot(time, translation_error);
set(gca, 'YScale', 'log')
ylabel('Translational error (meter)');
xlabel('Time (seconds)')

%% plot camera velocity
figure,
cam_velocity.time = zeros(N_steps, 1);
cam_velocity.l.x = zeros(N_steps, 1);
cam_velocity.l.y = zeros(N_steps, 1);
cam_velocity.l.z = zeros(N_steps, 1);
cam_velocity.a.x = zeros(N_steps, 1);
cam_velocity.a.y = zeros(N_steps, 1);
cam_velocity.a.z = zeros(N_steps, 1);
for i = 1:N_steps
    cam_vel = data{i}.cam_vel;
    cam_velocity.time(i) = data{i}.time;
    cam_velocity.l.x(i) = cam_vel(1);
    cam_velocity.l.y(i) = cam_vel(2);
    cam_velocity.l.z(i) = cam_vel(3);
    cam_velocity.a.x(i) = cam_vel(4);
    cam_velocity.a.y(i) = cam_vel(5);
    cam_velocity.a.z(i) = cam_vel(6);
end
subplot(1,2,1)
plot(cam_velocity.time, cam_velocity.l.x);
hold on
plot(cam_velocity.time, cam_velocity.l.y);
plot(cam_velocity.time, cam_velocity.l.z);
xlabel('time(sec)')
ylabel('velocity(m/s)');
title('camera linear velocity');
legend('x','y','z','Location','best');
subplot(1,2,2)
plot(cam_velocity.time, cam_velocity.a.x);
hold on
plot(cam_velocity.time, cam_velocity.a.y);
plot(cam_velocity.time, cam_velocity.a.z);
xlabel('time(sec)')
ylabel('velocity(rad/s)');
title('camera angular velocity');
legend('x','y','z','Location','best');

%% helper function
% Feel free to construct helper functions and put them here.

function cam_vel = PBVS_step(imgPoints, camera, featurePositions,...
                        goal_camera2board_pose, lambda, method)
    % pose estimation
    camera_param = cameraIntrinsics([camera.K(1,1), camera.K(2,2)],...
                                    [camera.K(1,3), camera.K(2,3)], ...
                                     camera.img_size);
    [board2camera_R, board2camera_t] = estimateWorldCameraPose(imgPoints',...
                                         featurePositions', camera_param,...
                                         'Confidence', 99.99,...
                                         'MaxReprojectionError', 1000);
    camera2board_pose = [board2camera_R, -board2camera_R*board2camera_t'; ...
                         0 0 0 1];
    % pbvs
    cam_vel = lambda * pbvs(camera2board_pose, goal_camera2board_pose, method);
end

function command_vel = pbvs(c2t, desired_c2t, method)
    % c2t is the pose from camera to target object
    % desired_c2t is the desired pose from camera to target object

    cstar2c = desired_c2t/c2t;
    R = cstar2c(1:3,1:3);   % rotation matrix
    t = cstar2c(1:3,4);     % translation matrix
    t_c2t = c2t(1:3,4);
    t_desired_c2t = desired_c2t(1:3,4);
    axang = rotm2axang(R);
    v = axang(1:3)';
    theta = axang(4);
    if method == 1
        v_c = -R'*t;
        omega_c = -theta*v;
    else
        % code here for other velocity control laws
    end

    command_vel = [v_c;omega_c];

        function S = skew(V)
            S = [0, -V(3), V(2); V(3), 0, -V(1); -V(2), V(1) 0];
        end
end
