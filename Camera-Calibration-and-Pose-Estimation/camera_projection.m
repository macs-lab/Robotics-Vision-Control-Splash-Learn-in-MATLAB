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