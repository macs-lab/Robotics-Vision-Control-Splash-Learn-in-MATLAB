classdef robotWithPen
    properties
        robot
        tcp2camera_pose
        tcp2pentip_pose
        camera_matrix
        img_size
        board_size
    end
    
    properties (GetAccess = 'private')
        base2board
    end
    
    methods
        function obj = robotWithPen(robot)
            obj.robot = robot;
            tcp2camera_quat = UnitQuaternion(0.999857,[-0.00456006, 0.0154471, -0.00523612]);
            tcp2camera_rot = R(tcp2camera_quat);
            tcp2camera_tvec = [0.0006608216698208661, -0.08493506686892241, 0.09476059359604611];
            obj.tcp2camera_pose = SE3(tcp2camera_rot, tcp2camera_tvec);
            obj.tcp2pentip_pose = SE3(0, 0, 0.135);
            obj.base2board = SE3(0.4, 0.1, 0.05);
            obj.camera_matrix = [1349.056772980567, 0, 774.9519749992618;
                                 0, 1351.558896220857, 557.5626838601945;
                                 0, 0, 1];
            obj.img_size = [1600, 1200];
            obj.board_size = [2*0.1, 0.1];
        end
        
        function imgPoints = project(obj, tcp_pose)
            % given pose of TCP w.r.t robot base, project feature point to
            % image plane.
            % input:
            %   tcp_pose (SE3): pose of TCP.
            % output:
            %   imgPoints (2x4): image coordinates of projected points.
            assert(isSE(tcp_pose), 'input argument tcp_pose is not SE3 object')
            camera2board = inv(obj.tcp2camera_pose) * inv(tcp_pose) * obj.base2board;
            %print(camera2board)
            L = obj.board_size(1);
            W = obj.board_size(2);
            objPoint_board = [0, L, L, 0; 0, 0, W, W; 0, 0, 0, 0];
            objPoint_camera = camera2board * objPoint_board;
            imgPoints_h = obj.camera_matrix * objPoint_camera;
            imgPoints = zeros(2, 4);
            imgPoints(1,:) = imgPoints_h(1,:) ./ imgPoints_h(3,:);
            imgPoints(2,:) = imgPoints_h(2,:) ./ imgPoints_h(3,:);
        end
        
        function plotImgPoint(obj, imgPoints)
            % draw img boundary
            width = obj.img_size(1);
            height = obj.img_size(2);
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
        
        function plotTrajectory(obj, trajectory)
            % trajectory: 1xN SE3. A list of waypoints, each waypoint
            % represent a TCP pose.
            
            % calculate the pen's pose.
            num_points = size(trajectory, 2);
            pen_traj = trajectory;
            points = zeros(3, num_points);
            for i = 1:num_points
                %pen_traj(i) = inv(obj.base2board) * trajectory(i) * obj.tcp2pentip_pose;
                pen_traj(i) = trajectory(i) * obj.tcp2pentip_pose;
                points(:,i) = pen_traj(i).t;
            end
            % plot trajectory of pen's tip.
            figure,
            hold on
            axis equal
            plot3(points(1,:), points(2,:), points(3,:), '*-', 'MarkerSize', 5);
            
            % draw board.
            L = obj.board_size(1);
            W = obj.board_size(2);
            point1 = obj.base2board.t;
            %point2 = obj.base2board.t + [L; 0; 0];
            point3 = obj.base2board.t + [L; W; 0];
            %point4 = obj.base2board.t + [0; W; 0];
            [X, Y] = meshgrid(point1(1):0.01:point3(1), point1(2):0.01:point3(2));
            Z = ones(size(X))*point1(3);
            surf(X, Y, Z, 'LineStyle', 'none')
            xlabel('x (meter)')
            ylabel('y (meter)')
            zlabel('z (meter)')
            view(45,25)
            title("Pen tip's moving trajectory")
        end
    end
end