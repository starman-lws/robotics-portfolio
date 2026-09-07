function rateMatrix = eulerRateMatrixZYX(attitudeRad)
%EULERRATEMATRIXZYX 将机体系角速度映射为 ZYX Euler 姿态角速度。
%
%   作者：Wenshao Lyu

    rollRad = attitudeRad(1);
    pitchRad = attitudeRad(2);
    cosinePitch = cos(pitchRad);

    % 限制 cos(pitch) 的最小幅值，避免接近 Euler 奇异点时除零。
    minimumCosineMagnitude = 1e-8;
    if abs(cosinePitch) < minimumCosineMagnitude
        cosinePitch = sign(cosinePitch + eps) * minimumCosineMagnitude;
    end

    tangentPitch = sin(pitchRad) / cosinePitch;
    inverseCosinePitch = 1 / cosinePitch;

    rateMatrix = [ ...
        1, sin(rollRad) * tangentPitch,    cos(rollRad) * tangentPitch; ...
        0, cos(rollRad),                  -sin(rollRad); ...
        0, sin(rollRad) * inverseCosinePitch, ...
           cos(rollRad) * inverseCosinePitch];
end
