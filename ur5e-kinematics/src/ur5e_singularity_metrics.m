function metrics = ur5e_singularity_metrics(q, tolerance)
%UR5E_SINGULARITY_METRICS 计算UR5e配置的数值奇异性指标。
%   q         : 1x6或6x1关节角向量，单位为degree。
%   tolerance : 可选的数值秩容差，默认值为1e-3。
%   metrics   : 包含Jacobian行列式、manipulability、奇异值、
%               最小奇异值、数值秩、条件数和解析奇异因子的结构体。

    if nargin < 2
        tolerance = 1e-3;
    end

    if ~isnumeric(tolerance) || ~isreal(tolerance) || ...
            ~isscalar(tolerance) || ~isfinite(tolerance) || tolerance < 0
        error('ur5e_singularity_metrics:InvalidTolerance', ...
              'tolerance必须是非负有限实数标量。');
    end

    J = ur5e_geometric_jacobian(q);
    singularValues = svd(J);
    minSingularValue = singularValues(end);

    % 奇异值乘积与sqrt(det(J*J.'))等价，并避免负数舍入误差。
    manipulability = prod(singularValues);

    if minSingularValue == 0
        conditionNumber = Inf;
    else
        conditionNumber = singularValues(1) / minSingularValue;
    end

    metrics.determinant = det(J);
    metrics.manipulability = manipulability;
    metrics.singularValues = singularValues;
    metrics.minSingularValue = minSingularValue;
    metrics.rank = sum(singularValues > tolerance);
    metrics.rankTolerance = tolerance;
    metrics.conditionNumber = conditionNumber;
    metrics.factors = ur5e_singularity_factors(q);
end
