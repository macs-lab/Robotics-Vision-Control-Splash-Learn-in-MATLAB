function [p_img] = cameraProjection(K,R,t,P_world)
%CAMERAPROJECTION Project 3D world point to image plane given camera model.
% input:
%   K: the 3x3 camera matrix
%   R: 3x3 rotation matrix
%   t: 3x1 translation vector
%   {R,t} compose the pose of camera coordinate system with respect to 
%       the world coordinate system.
%   P_world: 3xN vector. The coordinate of N points in the world coordinate
%       system
% output:
%   p_img: 2xN vector. The pixel coordinates of N points projected to the 
%       image plane.
assert(isequal(size(K),[3,3]));
assert(isequal(size(R),[3,3]));
assert(isequal(size(t),[3,1]));
assert(size(P_world,1) == 3);
num_P = size(P_world,2);

% your code start from here

%% create homogeneous transformation matrix
% pose of world (chessboard) w.r.t the camera
c2w_pose = eye(4);
c2w_pose(1:3, 1:3) = R';
c2w_pose(1:3, 4) = -R'*t;

%% transform p_world to camera coordinate system
P_world_h = [P_world; ones(1,num_P)];       % homogeneous coordinates
P_camera_h = c2w_pose * P_world_h;

%% camera projection
p_img_h = K*P_camera_h(1:3,:);              % homogeneous coordinates
p_img = zeros(2, num_P);
p_img(1,:) = p_img_h(1,:) ./ p_img_h(3,:);
p_img(2,:) = p_img_h(2,:) ./ p_img_h(3,:);

end

