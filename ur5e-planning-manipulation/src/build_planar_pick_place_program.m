function program = build_planar_pick_place_program( ...
        startPose, goalPose, plan)
%BUILD_PLANAR_PICK_PLACE_PROGRAM 将二维或SE(2)路径转换为平面拾放程序。
%   N x 2路径在运输时保持起始yaw；N x 3路径逐点使用theta。该函数
%   只生成TCP pose和waypoint，不连接或驱动机器人硬件。

    parameters = planar_pick_place_config();
    [startPosition, startTheta] = unpack_pose(startPose, 'startPose');
    [goalPosition, goalTheta] = unpack_pose(goalPose, 'goalPose');
    path = unpack_plan(plan);
    validate_path_endpoints(path, startPosition, startTheta, ...
        goalPosition, goalTheta, parameters);

    pickHeight = parameters.pickHeightMm;
    travelHeight = pickHeight + parameters.travelOffsetMm;
    startRotation = downward_tcp_rotation(startTheta);
    goalRotation = downward_tcp_rotation(goalTheta);

    program.prePickPose = [startPosition, travelHeight, startRotation];
    program.pickPose = [startPosition, pickHeight, startRotation];
    program.liftPose = program.prePickPose;
    program.transportWaypoints = build_transport_waypoints( ...
        path, travelHeight, startTheta, parameters);
    program.prePlacePose = [goalPosition, travelHeight, goalRotation];
    program.placePose = [goalPosition, pickHeight, goalRotation];
    program.retreatPose = program.prePlacePose;
    program.homePose = parameters.homePose;
    program.parameters = parameters;
end

function [position, theta] = unpack_pose(pose, argumentName)
    if ~isstruct(pose) || ~isscalar(pose) ...
            || ~all(isfield(pose, {'position', 'theta'}))
        error('build_planar_pick_place_program:InvalidPose', ...
              '%s必须包含position和theta。', argumentName);
    end

    position = pose.position;
    theta = pose.theta;
    if ~isnumeric(position) || ~isreal(position) ...
            || ~isequal(size(position), [1, 2]) ...
            || any(~isfinite(position(:))) ...
            || ~isnumeric(theta) || ~isreal(theta) ...
            || ~isscalar(theta) || ~isfinite(theta)
        error('build_planar_pick_place_program:InvalidPose', ...
              '%s必须提供有限的1x2 position和标量theta。', ...
              argumentName);
    end
    position = double(position);
    theta = double(theta);
end

function path = unpack_plan(plan)
    requiredFields = {'path', 'reachedGoal', 'stopReason', 'insideBoard'};
    if ~isstruct(plan) || ~isscalar(plan) ...
            || ~all(isfield(plan, requiredFields))
        error('build_planar_pick_place_program:InvalidPlan', ...
              'plan缺少通用平面拾放所需字段。');
    end

    path = plan.path;
    if ~isnumeric(path) || ~isreal(path) || ~ismatrix(path) ...
            || isempty(path) ...
            || ~(size(path, 2) == 2 || size(path, 2) == 3) ...
            || any(~isfinite(path(:)))
        error('build_planar_pick_place_program:InvalidPlan', ...
              'plan.path必须是非空N x 2或N x 3有限实数矩阵。');
    end
    if ~islogical(plan.reachedGoal) || ~isscalar(plan.reachedGoal) ...
            || ~islogical(plan.insideBoard) || ~isscalar(plan.insideBoard)
        error('build_planar_pick_place_program:InvalidPlan', ...
              'plan.reachedGoal和plan.insideBoard必须是逻辑标量。');
    end
    if ~(ischar(plan.stopReason) ...
            || (isstring(plan.stopReason) && isscalar(plan.stopReason)))
        error('build_planar_pick_place_program:InvalidPlan', ...
              'plan.stopReason必须是字符向量或字符串标量。');
    end
    if ~plan.reachedGoal || string(plan.stopReason) ~= "goal_reached" ...
            || ~plan.insideBoard
        error('build_planar_pick_place_program:PlanNotExecutable', ...
              '只有到达目标且位于工作区内的路径才能执行。');
    end
    path = double(path);
end

function validate_path_endpoints(path, startPosition, startTheta, ...
        goalPosition, goalTheta, parameters)
    positionTolerance = parameters.positionToleranceMm;
    if norm(path(1, 1:2) - startPosition) > positionTolerance ...
            || norm(path(end, 1:2) - goalPosition) > positionTolerance
        error('build_planar_pick_place_program:PosePlanMismatch', ...
              'plan.path的起终点位置与传入pose不一致。');
    end

    if size(path, 2) == 3
        startThetaError = angle_difference(path(1, 3), startTheta);
        goalThetaError = angle_difference(path(end, 3), goalTheta);
        if abs(startThetaError) > parameters.thetaToleranceRad ...
                || abs(goalThetaError) > parameters.thetaToleranceRad
            error('build_planar_pick_place_program:PosePlanMismatch', ...
                  'N x 3路径的起终点theta与传入pose不一致。');
        end
    end
end

function waypoints = build_transport_waypoints( ...
        path, travelHeight, startTheta, parameters)
    waypointCount = size(path, 1) - 1;
    if waypointCount == 0
        waypoints = zeros(0, 10);
        return;
    end

    if size(path, 2) == 2
        waypointTheta = repmat(startTheta, waypointCount, 1);
    else
        waypointTheta = path(2:end, 3);
    end
    rotations = zeros(waypointCount, 3);
    for waypointIndex = 1:waypointCount
        rotations(waypointIndex, :) = downward_tcp_rotation( ...
            waypointTheta(waypointIndex));
    end

    poses = [path(2:end, 1:2), ...
        repmat(travelHeight, waypointCount, 1), rotations];
    motion = repmat([parameters.jointAcceleration, ...
        parameters.jointVelocity, parameters.time, ...
        parameters.blendRadiusM], waypointCount, 1);
    motion(end, 4) = 0;
    waypoints = [poses, motion];
end

function difference = angle_difference(toAngle, fromAngle)
    difference = atan2( ...
        sin(toAngle - fromAngle), cos(toAngle - fromAngle));
end
