function [startPose, goalPose, obstacles, xLimits, yLimits] = ...
        large_item_scene_inputs(scene)
%LARGE_ITEM_SCENE_INPUTS 从视觉scene提取大物体SE(2)规划输入。

    requiredFields = {'bigItem', 'bigGoal', ...
                      'obstacles', 'boardReference'};
    if ~isstruct(scene) || ~isscalar(scene) ...
            || ~all(isfield(scene, requiredFields))
        error('large_item_scene_inputs:InvalidScene', ...
              'scene缺少大物体规划所需字段。');
    end

    startPose = unpack_pose(scene.bigItem, 'scene.bigItem');
    goalPose = unpack_pose(scene.bigGoal, 'scene.bigGoal');

    if ~isstruct(scene.obstacles) || ~isscalar(scene.obstacles) ...
            || ~isfield(scene.obstacles, 'position')
        error('large_item_scene_inputs:InvalidScene', ...
              'scene.obstacles必须包含position。');
    end
    obstacles = scene.obstacles.position;
    if isempty(obstacles)
        obstacles = zeros(0, 2);
    elseif ~isnumeric(obstacles) || ~isreal(obstacles) ...
            || ~ismatrix(obstacles) || size(obstacles, 2) ~= 2 ...
            || any(~isfinite(obstacles(:)))
        error('large_item_scene_inputs:InvalidScene', ...
              'scene.obstacles.position必须是有限实数N x 2矩阵。');
    else
        obstacles = double(obstacles);
    end

    try
        xLimits = double(scene.boardReference.XWorldLimits);
        yLimits = double(scene.boardReference.YWorldLimits);
    catch
        error('large_item_scene_inputs:InvalidBoardReference', ...
              'scene.boardReference必须提供XWorldLimits和YWorldLimits。');
    end
    if ~valid_limits(xLimits) || ~valid_limits(yLimits)
        error('large_item_scene_inputs:InvalidBoardReference', ...
              '工作区范围必须是递增的有限实数向量。');
    end
    xLimits = xLimits(:).';
    yLimits = yLimits(:).';
end

function pose = unpack_pose(value, argumentName)
    if ~isstruct(value) || ~isscalar(value) ...
            || ~all(isfield(value, {'position', 'theta'}))
        error('large_item_scene_inputs:InvalidScene', ...
              '%s必须包含position和theta。', argumentName);
    end
    position = value.position;
    theta = value.theta;
    if ~isnumeric(position) || ~isreal(position) ...
            || ~isequal(size(position), [1, 2]) ...
            || any(~isfinite(position(:))) ...
            || ~isnumeric(theta) || ~isreal(theta) ...
            || ~isscalar(theta) || ~isfinite(theta)
        error('large_item_scene_inputs:InvalidScene', ...
              '%s必须提供有限的1x2 position和标量theta。', argumentName);
    end
    pose = [double(position), double(theta)];
end

function valid = valid_limits(limits)
    valid = isnumeric(limits) && isreal(limits) ...
        && isvector(limits) && numel(limits) == 2 ...
        && all(isfinite(limits)) && limits(1) < limits(2);
end
