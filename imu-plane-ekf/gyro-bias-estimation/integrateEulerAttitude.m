function attitudeRad = integrateEulerAttitude( ...
    timeSec, gyroRadPerSec, initialAttitudeRad, biasRadPerSec)
%INTEGRATEEULERATTITUDE 根据机体系角速度积分 ZYX Euler 姿态角。
%   ATTITUDE = INTEGRATEEULERATTITUDE(TIME, GYRO, INITIALATTITUDE, BIAS)
%   对每个角速度采样采用零阶保持，并在后续时间区间内完成姿态积分。
%   所有角度量均使用 rad，时间使用 s。
%
%   输入
%     timeSec             包含 N 个严格递增时间戳的向量 [s]
%     gyroRadPerSec       3×N 机体系角速度测量值 [rad/s]
%     initialAttitudeRad  3×1 初始姿态 [roll; pitch; yaw] [rad]
%     biasRadPerSec       3×1 恒定陀螺仪偏置 [rad/s]
%
%   输出
%     attitudeRad         3×N 积分得到的 Euler 姿态角 [rad]
%
%   作者：Wenshao Lyu

    narginchk(4, 4);

    validateattributes(timeSec, {'double'}, ...
        {'vector', 'real', 'finite', 'nonempty'}, mfilename, 'timeSec', 1);
    validateattributes(gyroRadPerSec, {'double'}, ...
        {'2d', 'real', 'finite'}, mfilename, 'gyroRadPerSec', 2);
    validateattributes(initialAttitudeRad, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        mfilename, 'initialAttitudeRad', 3);
    validateattributes(biasRadPerSec, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        mfilename, 'biasRadPerSec', 4);

    timeSec = timeSec(:).';
    initialAttitudeRad = initialAttitudeRad(:);
    biasRadPerSec = biasRadPerSec(:);
    sampleCount = numel(timeSec);

    if ~isequal(size(gyroRadPerSec), [3, sampleCount])
        error('GyroBiasEstimation:InvalidGyroSize', ...
            'gyroRadPerSec must be 3-by-numel(timeSec).');
    end
    if any(diff(timeSec) <= 0)
        error('GyroBiasEstimation:InvalidTimeVector', ...
            'timeSec must be strictly increasing.');
    end

    attitudeRad = zeros(3, sampleCount);
    attitudeRad(:, 1) = initialAttitudeRad;

    % 在相邻采样区间内对去偏置角速度进行显式 Euler 积分。
    for sampleIndex = 2:sampleCount
        deltaTimeSec = timeSec(sampleIndex) - timeSec(sampleIndex - 1);
        correctedRateRadPerSec = ...
            gyroRadPerSec(:, sampleIndex - 1) - biasRadPerSec;
        rateMatrix = eulerRateMatrixZYX(attitudeRad(:, sampleIndex - 1));
        attitudeRad(:, sampleIndex) = attitudeRad(:, sampleIndex - 1) + ...
            deltaTimeSec * rateMatrix * correctedRateRadPerSec;
    end
end
