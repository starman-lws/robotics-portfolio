function force = apf_repulsive_force(position, obstacles, parameters)
%APF_REPULSIVE_FORCE 计算所有障碍物产生的人工势场排斥力。
%   position为一个Base平面坐标，obstacles为Nx2障碍物中心矩阵，单位为mm。

    position = normalise_point(position);
    obstacles = normalise_obstacles(obstacles);
    validate_parameters(parameters);
    force = zeros(1, 2);

    influenceDistance = parameters.repulsionInfluenceDistance;
    gain = parameters.repulsiveGain;
    maximumRepulsion = parameters.maximumRepulsion;

    for obstacleIndex = 1:size(obstacles, 1)
        difference = position - obstacles(obstacleIndex, :);
        distance = norm(difference);

        if distance >= influenceDistance
            obstacleForce = [0, 0];
        elseif distance > 1e-9
            direction = difference / distance;
            magnitude = gain ...
                * (1 / distance - 1 / influenceDistance) ...
                * (1 / distance^2);
            obstacleForce = min(magnitude, maximumRepulsion) ...
                * direction;
        else
            % 当前点与障碍物中心重合时，排斥方向没有唯一解。
            obstacleForce = [0, 0];
        end

        force = force + obstacleForce;
    end
end

function point = normalise_point(value)
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || ...
            numel(value) ~= 2 || any(~isfinite(value(:)))
        error('apf_repulsive_force:InvalidPoint', ...
              'position必须是包含两个有限实数的坐标向量。');
    end
    point = double(value(:).');
end

function obstacles = normalise_obstacles(value)
    if isempty(value)
        obstacles = zeros(0, 2);
        return;
    end
    if ~isnumeric(value) || ~isreal(value) || ~ismatrix(value) || ...
            size(value, 2) ~= 2 || any(~isfinite(value(:)))
        error('apf_repulsive_force:InvalidObstacles', ...
              'obstacles必须是Nx2有限实数矩阵或空数组。');
    end
    obstacles = double(value);
end

function validate_parameters(parameters)
    requiredFields = {'repulsionInfluenceDistance', ...
                      'repulsiveGain', 'maximumRepulsion'};
    if ~isstruct(parameters) || ~isscalar(parameters) || ...
            ~all(isfield(parameters, requiredFields))
        error('apf_repulsive_force:InvalidParameters', ...
              'parameters缺少排斥力参数。');
    end

    values = [parameters.repulsionInfluenceDistance, ...
              parameters.repulsiveGain, ...
              parameters.maximumRepulsion];
    if ~isnumeric(values) || ~isreal(values) || ...
            any(~isfinite(values)) || any(values <= 0)
        error('apf_repulsive_force:InvalidParameters', ...
              '排斥力参数必须是正的有限实数。');
    end
end
