function cosineValue = safeCosine(angleRad)
%SAFECOSINE 在 Euler 姿态奇异点附近限制余弦值的最小幅值。
%
%   作者：Wenshao Lyu

    cosineValue = cos(angleRad);
    minimumMagnitude = 1e-8;
    if abs(cosineValue) < minimumMagnitude
        cosineValue = sign(cosineValue + eps) * minimumMagnitude;
    end
end
