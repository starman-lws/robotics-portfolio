function force = apf_attractive_force(position, goal, parameters)
%APF_ATTRACTIVE_FORCE 计算人工势场中的目标吸引力。
%   position和goal是包含两个Base平面坐标的向量，单位为mm。

    position = normalise_point(position, 'position');
    goal = normalise_point(goal, 'goal');
    validate_parameters(parameters);

    difference = position - goal;
    distance = norm(difference);
    switchDistance = parameters.attractionSwitchDistance;
    gain = parameters.attractiveGain;

    if distance > switchDistance
        force = -switchDistance * gain * difference / distance;
    else
        force = -gain * difference;
    end
end

function point = normalise_point(value, argumentName)
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || ...
            numel(value) ~= 2 || any(~isfinite(value(:)))
        error('apf_attractive_force:InvalidPoint', ...
              '%s必须是包含两个有限实数的坐标向量。', argumentName);
    end
    point = double(value(:).');
end

function validate_parameters(parameters)
    requiredFields = {'attractionSwitchDistance', 'attractiveGain'};
    if ~isstruct(parameters) || ~isscalar(parameters) || ...
            ~all(isfield(parameters, requiredFields))
        error('apf_attractive_force:InvalidParameters', ...
              'parameters缺少吸引力参数。');
    end

    values = [parameters.attractionSwitchDistance, ...
              parameters.attractiveGain];
    if ~isnumeric(values) || ~isreal(values) || ...
            any(~isfinite(values)) || any(values <= 0)
        error('apf_attractive_force:InvalidParameters', ...
              '吸引力参数必须是正的有限实数。');
    end
end
