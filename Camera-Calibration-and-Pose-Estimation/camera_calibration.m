% UW Mechanical Engineering
% Robotics, Visiton, and Mechatronics for Manufacturing, Sp 2021
% HW3 / Lab2
% Matlab toolbox required: computer vision toolbox 
% (https://www.mathworks.com/help/vision/index.html)
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
