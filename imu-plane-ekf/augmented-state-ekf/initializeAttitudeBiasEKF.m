function [state, covariance] = initializeAttitudeBiasEKF( ...
    initialAttitudeRad, initialBiasRadPerSec, ...
    attitudeStdRad, biasStdRadPerSec)
%INITIALIZEATTITUDEBIASEKF 初始化六维姿态—偏置 EKF。
%   状态排列为
%       [roll; pitch; yaw; biasX; biasY; biasZ]
%   姿态使用 rad，陀螺仪偏置使用 rad/s。
%
%   输入
%     initialAttitudeRad    3×1 初始 ZYX Euler 姿态角 [rad]
%     initialBiasRadPerSec  3×1 初始陀螺仪偏置 [rad/s]
%     attitudeStdRad        3×1 姿态标准差 [rad]
%     biasStdRadPerSec      3×1 偏置标准差 [rad/s]
%
%   输出
%     state                 6×1 初始姿态—偏置状态
%     covariance            6×6 初始状态协方差矩阵
%
%   作者：Wenshao Lyu

    narginchk(4, 4);

    validateattributes(initialAttitudeRad, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        mfilename, 'initialAttitudeRad', 1);
    validateattributes(initialBiasRadPerSec, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        mfilename, 'initialBiasRadPerSec', 2);
    validateattributes(attitudeStdRad, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite', 'nonnegative'}, ...
        mfilename, 'attitudeStdRad', 3);
    validateattributes(biasStdRadPerSec, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite', 'nonnegative'}, ...
        mfilename, 'biasStdRadPerSec', 4);

    % 假设各初始状态分量相互独立，以标准差平方构造对角协方差。
    state = [initialAttitudeRad(:); initialBiasRadPerSec(:)];
    stateStandardDeviation = [attitudeStdRad(:); biasStdRadPerSec(:)];
    covariance = diag(stateStandardDeviation .^ 2);
end
