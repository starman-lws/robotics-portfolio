function [unitVector, isValid] = normalizeVectorSafe(vector)
%NORMALIZEVECTORSAFE 在检测退化情况后安全地归一化有限向量。
%
%   作者：Wenshao Lyu

    unitVector = vector(:);
    vectorNorm = norm(unitVector);
    isValid = all(isfinite(unitVector)) && vectorNorm > 1e-12;

    if isValid
        unitVector = unitVector / vectorNorm;
    end
end
