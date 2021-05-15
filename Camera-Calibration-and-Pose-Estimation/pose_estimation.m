%% load image sequence
v = VideoReader('input.avi');

%% estimate camera pose for each frame
num_frame = v.NumFrames;
worldPoints_3D = [worldPoints, zeros(size(worldPoints, 1), 1)];
cameraOrientations = zeros(3,3,num_frame);
cameraLocations = zeros(1,3,num_frame);
for i = 1:num_frame
    frame = read(v,i);
    [imagePoints, ~] = detectCheckerboardPoints(frame);
    [cameraOrientations(:,:,i), cameraLocations(:,:,i)] = ... 
        estimateWorldCameraPose(imagePoints, worldPoints_3D, params);
    % NOTE:
    % 1. the orientation and locations returned here are the pose of camera 
    % with respect to the chessboard.
    % 2. MATLAB tends to use row vector to represent coordinates instead of
    % column vector as what we see in the textbook and OpenCV. As a
    % result, rotation matrix returned here is transposed, compared with
    % the rotation matrix defined in textbook and OpenCV. To transform
    % coordinates from one coordinates to another, MATLAB use
    %                   v1 = v2 * R
    % Note the differense. In the textbook and OpenCV, we use
    %                   v1 = R * v2    
end

%% plot camera moving trajectory
figure,
hold on
axis equal
for i = 1:num_frame
    plotCamera('Size', 0.005, 'Orientation', cameraOrientations(:,:,i), ...
        'Location', cameraLocations(:,:,i))
end
addpath('plotcube')
plotcube([6*0.022, 9*0.022, 0.001], [0,0,0], 1, [0, 1, 0])
xlabel('x (meter)')
ylabel('y (meter)')
zlabel('z (meter)')