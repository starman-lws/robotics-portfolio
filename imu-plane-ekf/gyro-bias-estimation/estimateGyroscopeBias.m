function [biasRadPerSec, attitudeEstimateRad, diagnostics] = estimateGyroscopeBias( ...
    timeSec, gyroRadPerSec, initialAttitudeRad, measurementIndices, ...
    measuredAttitudeRad, initialBiasRadPerSec)
%ESTIMATEGYROSCOPEBIAS 估计三轴恒定陀螺仪偏置。
%   [BIAS, ATTITUDE, DIAGNOSTICS] = ESTIMATEGYROSCOPEBIAS(TIME, GYRO,
%   INITIALATTITUDE, INDICES, MEASUREDATTITUDE, INITIALBIAS) 通过最小化
%   指定观测时刻的姿态角周期残差平方和，估计恒定偏置。
%
%   输入
%     timeSec              包含 N 个时间戳的向量 [s]
%     gyroRadPerSec        3×N 三轴角速度测量值 [rad/s]
%     initialAttitudeRad   3×1 初始 ZYX Euler 姿态角 [rad]
%     measurementIndices   M 个姿态观测对应的采样索引
%     measuredAttitudeRad  3×M 姿态观测 [rad]；按分量忽略 NaN
%     initialBiasRadPerSec 可选的 3×1 优化初值 [rad/s]
%
%   输出
%     biasRadPerSec        估计得到的三轴恒定偏置 [rad/s]
%     attitudeEstimateRad  3×N 重建的 Euler 姿态角 [rad]
%     diagnostics          优化器状态和最终残差代价
%
%   姿态排列为 [roll; pitch; yaw]，使用 ZYX Euler 运动学将机体系
%   角速度映射为姿态角速度。
%
%   作者：Wenshao Lyu

    narginchk(5, 6);

    validateattributes(timeSec, {'double'}, ...
        {'vector', 'real', 'finite', 'nonempty'}, mfilename, 'timeSec', 1);
    validateattributes(gyroRadPerSec, {'double'}, ...
        {'2d', 'real', 'finite'}, mfilename, 'gyroRadPerSec', 2);
    validateattributes(initialAttitudeRad, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        mfilename, 'initialAttitudeRad', 3);
    validateattributes(measurementIndices, {'double'}, ...
        {'vector', 'real', 'finite', 'integer', 'positive'}, ...
        mfilename, 'measurementIndices', 4);
    validateattributes(measuredAttitudeRad, {'double'}, ...
        {'2d', 'real'}, mfilename, 'measuredAttitudeRad', 5);

    if nargin < 6 || isempty(initialBiasRadPerSec)
        initialBiasRadPerSec = zeros(3, 1);
    end
    validateattributes(initialBiasRadPerSec, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        mfilename, 'initialBiasRadPerSec', 6);

    timeSec = timeSec(:).';
    initialAttitudeRad = initialAttitudeRad(:);
    measurementIndices = measurementIndices(:).';
    initialBiasRadPerSec = initialBiasRadPerSec(:);

    sampleCount = numel(timeSec);
    measurementCount = numel(measurementIndices);
    if ~isequal(size(gyroRadPerSec), [3, sampleCount])
        error('GyroBiasEstimation:InvalidGyroSize', ...
            'gyroRadPerSec must be 3-by-numel(timeSec).');
    end
    if ~isequal(size(measuredAttitudeRad), [3, measurementCount])
        error('GyroBiasEstimation:InvalidMeasurementSize', ...
            'measuredAttitudeRad must be 3-by-numel(measurementIndices).');
    end
    if any(~(isfinite(measuredAttitudeRad(:)) | isnan(measuredAttitudeRad(:))))
        error('GyroBiasEstimation:InvalidMeasurementValue', ...
            'Attitude observations may contain finite values or NaN only.');
    end
    if any(diff(timeSec) <= 0)
        error('GyroBiasEstimation:InvalidTimeVector', ...
            'timeSec must be strictly increasing.');
    end
    if any(measurementIndices > sampleCount) || ...
            any(diff(measurementIndices) <= 0)
        error('GyroBiasEstimation:InvalidMeasurementIndices', ...
            'Measurement indices must be unique, increasing, and in range.');
    end
    if ~any(isfinite(measuredAttitudeRad(:)))
        error('GyroBiasEstimation:NoValidObservations', ...
            'At least one finite attitude observation is required.');
    end

    % 使用稀疏姿态观测构造偏置优化目标函数。
    objective = @(candidateBias) attitudeResidualCost( ...
        candidateBias, timeSec, gyroRadPerSec, initialAttitudeRad, ...
        measurementIndices, measuredAttitudeRad);

    optimizerOptions = optimset( ...
        'Display', 'off', ...
        'MaxFunEvals', 2000, ...
        'MaxIter', 1000, ...
        'TolFun', 1e-12, ...
        'TolX', 1e-10);

    % 通过无导数单纯形搜索求解三轴恒定偏置。
    [biasRadPerSec, finalCost, exitFlag, optimizerOutput] = fminsearch( ...
        objective, initialBiasRadPerSec, optimizerOptions);
    biasRadPerSec = biasRadPerSec(:);

    attitudeEstimateRad = integrateEulerAttitude( ...
        timeSec, gyroRadPerSec, initialAttitudeRad, biasRadPerSec);

    diagnostics = struct( ...
        'finalCost', finalCost, ...
        'exitFlag', exitFlag, ...
        'optimizerOutput', optimizerOutput, ...
        'validResidualCount', nnz(isfinite(measuredAttitudeRad)));
end
