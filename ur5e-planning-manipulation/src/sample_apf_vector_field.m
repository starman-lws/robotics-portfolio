function field = sample_apf_vector_field( ...
        goal, obstacles, xLimits, yLimits, parameters)
%SAMPLE_APF_VECTOR_FIELD 在指定Base工作区采样人工势场向量。
%   field包含同尺寸的X、Y、U和V矩阵，位置单位为mm。

    xLimits = normalise_limits(xLimits, 'xLimits');
    yLimits = normalise_limits(yLimits, 'yLimits');

    if ~isstruct(parameters) || ~isscalar(parameters) || ...
            ~isfield(parameters, 'gridSpacing') || ...
            ~isnumeric(parameters.gridSpacing) || ...
            ~isreal(parameters.gridSpacing) || ...
            ~isscalar(parameters.gridSpacing) || ...
            ~isfinite(parameters.gridSpacing) || ...
            parameters.gridSpacing <= 0
        error('sample_apf_vector_field:InvalidParameters', ...
              'parameters.gridSpacing必须是正的有限实数标量。');
    end

    xPoints = xLimits(1):parameters.gridSpacing:xLimits(2);
    yPoints = yLimits(1):parameters.gridSpacing:yLimits(2);
    [X, Y] = meshgrid(xPoints, yPoints);
    U = zeros(size(X));
    V = zeros(size(Y));

    for sampleIndex = 1:numel(X)
        force = apf_total_force( ...
            [X(sampleIndex), Y(sampleIndex)], ...
            goal, obstacles, parameters);
        U(sampleIndex) = force(1);
        V(sampleIndex) = force(2);
    end

    field.X = X;
    field.Y = Y;
    field.U = U;
    field.V = V;
end

function limits = normalise_limits(value, argumentName)
    if ~isnumeric(value) || ~isreal(value) || ~isvector(value) || ...
            numel(value) ~= 2 || any(~isfinite(value(:)))
        error('sample_apf_vector_field:InvalidLimits', ...
              '%s必须是包含两个有限实数的范围向量。', argumentName);
    end

    limits = double(value(:).');
    if limits(1) >= limits(2)
        error('sample_apf_vector_field:InvalidLimits', ...
              '%s的下限必须小于上限。', argumentName);
    end
end
