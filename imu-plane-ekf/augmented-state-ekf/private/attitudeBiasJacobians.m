function [stateTransition, noiseGain] = attitudeBiasJacobians( ...
    state, gyroMeasurementRadPerSec, deltaTimeSec)
%ATTITUDEBIASJACOBIANS 构造状态与输入的解析 Jacobian。
%
%   作者：Wenshao Lyu

    rollRad = state(1);
    pitchRad = state(2);
    % 使用当前偏置估计恢复真实角速度。
    correctedRate = gyroMeasurementRadPerSec(:) - state(4:6);
    rateY = correctedRate(2);
    rateZ = correctedRate(3);

    cosinePitch = safeCosine(pitchRad);
    sinePitch = sin(pitchRad);
    tangentPitch = sinePitch / cosinePitch;
    inverseCosinePitch = 1 / cosinePitch;
    secantSquaredPitch = inverseCosinePitch ^ 2;

    rateMatrix = [ ...
        1, sin(rollRad) * tangentPitch,    cos(rollRad) * tangentPitch; ...
        0, cos(rollRad),                  -sin(rollRad); ...
        0, sin(rollRad) * inverseCosinePitch, ...
           cos(rollRad) * inverseCosinePitch];

    attitudeRateJacobian = [ ...
        (rateY * cos(rollRad) - rateZ * sin(rollRad)) * tangentPitch, ...
        (rateY * sin(rollRad) + rateZ * cos(rollRad)) * secantSquaredPitch, 0; ...
        -rateY * sin(rollRad) - rateZ * cos(rollRad), 0, 0; ...
        (rateY * cos(rollRad) - rateZ * sin(rollRad)) * inverseCosinePitch, ...
        (rateY * sin(rollRad) + rateZ * cos(rollRad)) * ...
            sinePitch * secantSquaredPitch, 0];

    % 对连续时间姿态模型进行一阶离散化，并保留常值偏置状态。
    stateTransition = eye(6);
    stateTransition(1:3, 1:3) = eye(3) + ...
        deltaTimeSec * attitudeRateJacobian;
    stateTransition(1:3, 4:6) = -deltaTimeSec * rateMatrix;

    % 将三轴陀螺仪噪声映射至六维增广状态。
    noiseGain = zeros(6, 3);
    noiseGain(1:3, :) = deltaTimeSec * rateMatrix;
end
