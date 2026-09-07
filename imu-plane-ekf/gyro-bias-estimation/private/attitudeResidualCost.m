function cost = attitudeResidualCost( ...
    candidateBiasRadPerSec, timeSec, gyroRadPerSec, initialAttitudeRad, ...
    measurementIndices, measuredAttitudeRad)
%ATTITUDERESIDUALCOST 计算用于偏置估计的周期姿态残差代价。
%
%   作者：Wenshao Lyu

    estimatedAttitudeRad = integrateEulerAttitude( ...
        timeSec, gyroRadPerSec, initialAttitudeRad, ...
        candidateBiasRadPerSec(:));

    % 将角度误差包裹到主值区间，避免跨越 ±pi 时产生虚假大残差。
    residualRad = estimatedAttitudeRad(:, measurementIndices) - ...
        measuredAttitudeRad;
    residualRad = atan2(sin(residualRad), cos(residualRad));
    validResiduals = residualRad(isfinite(measuredAttitudeRad));
    cost = sum(validResiduals .^ 2);
end
