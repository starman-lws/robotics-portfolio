function [qMin, index, metrics] = ...
    find_min_manipulability_configuration(jointPoses, tolerance)
%FIND_MIN_MANIPULABILITY_CONFIGURATION 查找manipulability最小的配置。
%   jointPoses : Nx6关节配置矩阵，单位为degree。
%   tolerance  : 可选的数值秩容差，默认值为1e-3。
%   qMin       : 第一组达到最小manipulability的1x6关节配置。
%   index      : qMin在输入矩阵中的行号。
%   metrics    : qMin对应的完整奇异性诊断结果。

    if nargin < 2
        tolerance = 1e-3;
    end

    if ~isnumeric(jointPoses) || ~isreal(jointPoses) || ...
            ~ismatrix(jointPoses) || isempty(jointPoses) || ...
            size(jointPoses,2) ~= 6 || any(~isfinite(jointPoses(:)))
        error('find_min_manipulability_configuration:InvalidJointMatrix', ...
              'jointPoses必须是非空的Nx6有限实数矩阵。');
    end

    if ~isnumeric(tolerance) || ~isreal(tolerance) || ...
            ~isscalar(tolerance) || ~isfinite(tolerance) || tolerance < 0
        error('find_min_manipulability_configuration:InvalidTolerance', ...
              'tolerance必须是非负有限实数标量。');
    end

    configurationCount = size(jointPoses, 1);
    manipulabilityValues = zeros(configurationCount, 1);

    for configurationIndex = 1:configurationCount
        currentMetrics = ur5e_singularity_metrics( ...
            jointPoses(configurationIndex,:), tolerance);
        manipulabilityValues(configurationIndex) = ...
            currentMetrics.manipulability;
    end

    [~, index] = min(manipulabilityValues);
    qMin = jointPoses(index,:);
    metrics = ur5e_singularity_metrics(qMin, tolerance);
end
