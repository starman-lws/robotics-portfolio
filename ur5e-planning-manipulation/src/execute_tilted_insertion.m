function execute_tilted_insertion(rtde, vacuum, program)
%EXECUTE_TILTED_INSERTION 使用RTDE和真空吸盘执行倾斜插入程序。
%   program由BUILD_TILTED_INSERTION_PROGRAM生成；硬件错误直接传递。

    validate_program(program);
    parameters = program.parameters;

    move_joint(rtde, program.prePickPose, parameters);
    pause(parameters.motionPauseS);
    move_linear(rtde, program.pickPose, parameters);
    pause(parameters.motionPauseS);
    vacuum.grip();
    pause(parameters.gripPauseS);
    move_linear(rtde, program.liftPose, parameters);
    pause(parameters.motionPauseS);

    execute_transport(rtde, program.transportWaypoints);
    if ~isempty(program.transportWaypoints)
        pause(parameters.motionPauseS);
    end

    move_linear(rtde, program.preAlignPose, parameters);
    pause(parameters.motionPauseS);
    move_linear(rtde, program.tiltClearancePose, parameters);
    pause(parameters.motionPauseS);
    move_linear(rtde, program.slotApproachPose, parameters);
    pause(parameters.motionPauseS);
    move_linear(rtde, program.insertPose, parameters);
    pause(parameters.motionPauseS);
    vacuum.release();
    pause(parameters.releasePauseS);
    move_linear(rtde, program.retreatPose, parameters);
    pause(parameters.motionPauseS);
    move_joint(rtde, program.homePose, parameters);
end

function move_joint(rtde, pose, parameters)
    rtde.movej(pose, 'pose', parameters.jointAcceleration, ...
        parameters.jointVelocity, parameters.time, 0);
end

function move_linear(rtde, pose, parameters)
    rtde.movel(pose, 'pose', parameters.linearAcceleration, ...
        parameters.linearVelocity, parameters.time, 0);
end

function execute_transport(rtde, waypoints)
    waypointCount = size(waypoints, 1);
    if waypointCount == 0
        return;
    end
    if waypointCount == 1
        waypoint = waypoints(1, :);
        rtde.movej(waypoint(1:6), 'pose', ...
            waypoint(7), waypoint(8), waypoint(9), waypoint(10));
    else
        rtde.movej(waypoints, 'pose');
    end
end

function validate_program(program)
    poseFields = {'prePickPose', 'pickPose', 'liftPose', ...
        'preAlignPose', 'tiltClearancePose', 'slotApproachPose', ...
        'insertPose', 'retreatPose', 'homePose'};
    requiredFields = [poseFields, {'transportWaypoints', 'parameters'}];
    if ~isstruct(program) || ~isscalar(program) ...
            || ~all(isfield(program, requiredFields))
        invalid_program('program缺少倾斜插入执行所需字段。');
    end
    for fieldIndex = 1:numel(poseFields)
        pose = program.(poseFields{fieldIndex});
        if ~isnumeric(pose) || ~isreal(pose) ...
                || ~isequal(size(pose), [1, 6]) ...
                || any(~isfinite(pose(:)))
            invalid_program('%s必须是1x6有限实数向量。', ...
                poseFields{fieldIndex});
        end
    end

    waypoints = program.transportWaypoints;
    if ~isnumeric(waypoints) || ~isreal(waypoints) ...
            || ~ismatrix(waypoints) || size(waypoints, 2) ~= 10 ...
            || any(~isfinite(waypoints(:)))
        invalid_program('transportWaypoints必须是N x 10有限实数矩阵。');
    end
    if ~isempty(waypoints) && abs(waypoints(end, 10)) > 1e-12
        invalid_program('最后一个transport waypoint的blend必须为零。');
    end

    parameters = program.parameters;
    requiredParameters = {'jointAcceleration', 'jointVelocity', ...
        'linearAcceleration', 'linearVelocity', 'time', ...
        'motionPauseS', 'gripPauseS', 'releasePauseS'};
    if ~isstruct(parameters) || ~isscalar(parameters) ...
            || ~all(isfield(parameters, requiredParameters))
        invalid_program('program.parameters缺少执行参数。');
    end
    positiveValues = [parameters.jointAcceleration, ...
        parameters.jointVelocity, parameters.linearAcceleration, ...
        parameters.linearVelocity];
    nonnegativeValues = [parameters.time, parameters.motionPauseS, ...
        parameters.gripPauseS, parameters.releasePauseS];
    if any(~isfinite(positiveValues)) || any(positiveValues <= 0) ...
            || any(~isfinite(nonnegativeValues)) ...
            || any(nonnegativeValues < 0)
        invalid_program('执行参数必须是有效的有限实数。');
    end
end

function invalid_program(message, varargin)
    error('execute_tilted_insertion:InvalidProgram', ...
          message, varargin{:});
end
