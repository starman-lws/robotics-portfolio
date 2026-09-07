function [predictedState, predictedCovariance] = predictAttitudeBiasEKF( ...
    priorState, priorCovariance, gyroMeasurementRadPerSec, ...
    deltaTimeSec, gyroNoiseStdRadPerSec)
%PREDICTATTITUDEBIASEKF 执行六维姿态—偏置 EKF 预测。
%   陀螺仪偏置采用常值状态模型，白噪声通过非线性 ZYX Euler
%   运动学映射传播至状态协方差。
%
%   输入
%     priorState                 6×1 先验姿态—偏置状态
%     priorCovariance            6×6 先验状态协方差
%     gyroMeasurementRadPerSec   3×1 机体系角速度测量值 [rad/s]
%     deltaTimeSec               正数预测时间间隔 [s]
%     gyroNoiseStdRadPerSec      标量陀螺仪噪声标准差 [rad/s]
%
%   输出
%     predictedState             6×1 预测状态
%     predictedCovariance        6×6 预测协方差
%
%   作者：Wenshao Lyu

    narginchk(5, 5);

    validateattributes(priorState, {'double'}, ...
        {'vector', 'numel', 6, 'real', 'finite'}, ...
        mfilename, 'priorState', 1);
    validateattributes(priorCovariance, {'double'}, ...
        {'size', [6, 6], 'real', 'finite'}, ...
        mfilename, 'priorCovariance', 2);
    validateattributes(gyroMeasurementRadPerSec, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        mfilename, 'gyroMeasurementRadPerSec', 3);
    validateattributes(deltaTimeSec, {'double'}, ...
        {'scalar', 'real', 'finite', 'positive'}, ...
        mfilename, 'deltaTimeSec', 4);
    validateattributes(gyroNoiseStdRadPerSec, {'double'}, ...
        {'scalar', 'real', 'finite', 'nonnegative'}, ...
        mfilename, 'gyroNoiseStdRadPerSec', 5);

    priorState = priorState(:);
    gyroMeasurementRadPerSec = gyroMeasurementRadPerSec(:);
    % 从角速度测量中扣除当前偏置估计。
    correctedAngularRate = gyroMeasurementRadPerSec - priorState(4:6);

    [~, attitudeRate] = attitudeKinematicsZYX( ...
        priorState(1:3), correctedAngularRate);
    [stateTransition, noiseGain] = attitudeBiasJacobians( ...
        priorState, gyroMeasurementRadPerSec, deltaTimeSec);

    predictedState = priorState;
    predictedState(1:3) = priorState(1:3) + ...
        deltaTimeSec * attitudeRate;

    % 通过输入 Jacobian 将陀螺仪白噪声传播至状态空间。
    gyroNoiseCovariance = ...
        (gyroNoiseStdRadPerSec ^ 2) * eye(3);
    processCovariance = noiseGain * gyroNoiseCovariance * noiseGain.';
    predictedCovariance = stateTransition * priorCovariance * ...
        stateTransition.' + processCovariance;
    predictedCovariance = symmetrizeCovariance(predictedCovariance);
end
