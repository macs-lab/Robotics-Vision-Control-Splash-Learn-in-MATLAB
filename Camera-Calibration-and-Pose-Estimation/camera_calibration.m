%% load chessboard images
images = imageSet(fullfile('calibration_images'));
imageFileNames = images.ImageLocation;

%% detect image feature points
[imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);

%% set world points
squareSize = 0.022;
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

%% calibrate camera
I = readimage(images,1); 
imageSize = [size(I, 1),size(I, 2)];
params = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize, ...
                                  'WorldUnits', 'm');
K = params.IntrinsicMatrix';
% See textbook (11.7) for matrix K's defination

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
    plotCamera('Size', 0.005, 'Orientation', cameraOrientations(:,:,i), 'Location', cameraLocations(:,:,i))
end
addpath('plotcube')
plotcube([6*0.022, 9*0.022, 0.001], [0,0,0], 1, [0, 1, 0])
xlabel('x (meter)')
ylabel('y (meter)')
zlabel('z (meter)')
%% draw x y z axis to each frame
axisPoints = [0, 0, 0; 0.088, 0, 0; 0, 0.088, 0; 0, 0, 0.088]';
v_out = VideoWriter('output','MPEG-4');
open(v_out)
for i = 1:num_frame
    frame = read(v, i);
    R = cameraOrientations(:,:,i)';
    t = cameraLocations(:,:,i)';
    % Note: same as in the textbook, we use column vector to represent 
    % coordinates.
    %% draw detected feature points
    [imagePoints, ~] = detectCheckerboardPoints(frame);
    frame = insertText(frame, imagePoints, 1:size(imagePoints,1));
    frame = insertShape(frame, "circle", ...
        [imagePoints, ones(size(imagePoints,1),1)*5], ...
        "Color", "red");
    %% draw projected x,y,z axis
    K = params.IntrinsicMatrix';
    imgPoints = cameraProjection(K, R, t, axisPoints);
    frame = insertShape(frame,"Line",[imgPoints(:,1)' imgPoints(:,2)'; ...
            imgPoints(:,1)' imgPoints(:,3)'; imgPoints(:,1)' imgPoints(:,4)'], ...
            "Color",["red","green","blue"],"LineWidth",7);
    writeVideo(v_out, frame);
end
close(v_out)