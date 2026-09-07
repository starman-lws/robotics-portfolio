function [rateMatrix, attitudeRate] = ...
    attitudeKinematicsZYX(attitudeRad, angularRateBodyRadPerSec)
%ATTITUDEKINEMATICSZYX 计算非线性 ZYX Euler 姿态运动学。
%
%   作者：Wenshao Lyu

    rollRad = attitudeRad(1);
    pitchRad = attitudeRad(2);
    % 对接近俯仰奇异点的分母进行数值保护。
    cosinePitch = safeCosine(pitchRad);
    tangentPitch = sin(pitchRad) / cosinePitch;
    inverseCosinePitch = 1 / cosinePitch;

    rateMatrix = [ ...
        1, sin(rollRad) * tangentPitch,    cos(rollRad) * tangentPitch; ...
        0, cos(rollRad),                  -sin(rollRad); ...
        0, sin(rollRad) * inverseCosinePitch, ...
           cos(rollRad) * inverseCosinePitch];

    attitudeRate = rateMatrix * angularRateBodyRadPerSec(:);
end
