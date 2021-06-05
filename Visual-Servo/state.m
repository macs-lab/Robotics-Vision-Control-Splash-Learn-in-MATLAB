classdef state
    properties
        cameraPose        % pose of camera w.r.t board (4x4 matrix)
        camera             % camera model
        N                  % number of feature points
        featurePoints      % coordinate of feature points in board frame (3xN matrix)
        imagePoints        % coordinate of projected porints (2xN matrix)
        time               % timestamp (double) (seconds)
        cam_vel            % camera velocity (6x1 matrix)
    end
    
    methods
        function obj = state(init_camera_pose, cam_model, featurePoints)
            % initialization method
            obj.cameraPose = init_camera_pose;
            obj.camera = cam_model;
            obj.featurePoints = featurePoints;
            obj.time = 0;
            obj.N = size(featurePoints, 2);
            obj.imagePoints = zeros(2, obj.N);
            obj.cam_vel = zeros(6,1);
        end
        
        function obj = step(obj, vel, Ts)
            % update camera pose assumming a piecewise-constant camera
            % velocity.
            % input:
            %   vel (6x1): camera velocity w.r.t the camera frame.
            %   Ts (double): step size in time (seconds)
            % output:
            %   obj (state): new state object with the updated camera pose.
            obj.cameraPose = trnorm(obj.cameraPose*delta2tr(vel*Ts));
            obj.time = obj.time + Ts;
            obj.cam_vel = vel;
        end
        
        function [obj, imgPoints] = project(obj)
            % generate projected feature points on the image plane
            % input:
            %   noise (double): std of noise.
            % output:
            %   imgPoints (2xN): image coordinates of projected points.
            imgPoints = cameraProjection(obj.camera.K, ...
                                         obj.cameraPose(1:3, 1:3),...
                                         obj.cameraPose(1:3, 4),...
                                         obj.featurePoints);
            obj.imagePoints = imgPoints;
        end
        
        function Z = getFeatureDepth(obj)
            % Give the true depth of each feature point w.r.t the camera
            % frame.
            % output:
            %   Z (1xN): feature depth for each feature point
    
            featurePos_h = [obj.featurePoints; ones(1,obj.N)];
            featurePos_cam = obj.cameraPose \ featurePos_h;
            Z = featurePos_cam(3,:);
        end
        
        function plotImgPoint(obj, imgPoints)
            % Visulize the imgPoints in a image plane.
            % used for debug purpose
            
            % draw img boundary
            width = obj.camera.img_size(1);
            height = obj.camera.img_size(2);
            figure,
            plot([0, width], [0, 0], 'k-')
            axis equal
            hold on
            plot([width, width], [0, -height], 'k-')
            plot([width, 0], [-height, -height], 'k-')
            plot([0, 0], [-height, 0], 'k-')
            xlim([0, width])
            ylim([-height, 0])
            
            % draw img points
            plot(imgPoints(1,:), -imgPoints(2,:), '*')
        end
    end
end