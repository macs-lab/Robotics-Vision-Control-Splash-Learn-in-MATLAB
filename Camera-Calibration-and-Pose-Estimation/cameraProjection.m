function [p_img] = cameraProjection(K,R,t,P_world)
%CAMERAPROJECTION Project 3D world point to image plane given camera model.
%   K: the 3x3 camera matrix
%   R: 3x3 rotation matrix
%   t: 3x1 translation vector
%   {R,t} compose the pose of camera coordinate system with respect to 
%       the chessboard coordinate system.
%   P_world: 3xN vector. The coordinate of N points in the chessboard coordinate
%       system
%   p_img: 2xN vector. The pixel coordinates of N points projected to the 
%       image plane.
assert(isequal(size(K),[3,3]));
assert(isequal(size(R),[3,3]));
assert(isequal(size(t),[3,1]));
assert(size(P_world,1) == 3);
num_P = size(P_world,2);
% initialize return
p_img = zeros(2, num_P);

% your code start from here

end

