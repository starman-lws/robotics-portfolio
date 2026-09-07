function [referenceIndex, minimumAngleRad, globalNormal] = ...
    associatePlaneNormal(localNormal, attitudeRad, referenceNormals, toleranceRad)
%ASSOCIATEPLANENORMAL 将局部平面法向量关联到已知参考方向。
%   [INDEX, ANGLE, GLOBALNORMAL] = ASSOCIATEPLANENORMAL(LOCALNORMAL,
%   ATTITUDE, REFERENCENORMALS, TOLERANCE) 将 LOCALNORMAL 从机体系旋转至
%   全局坐标系，并按照有向夹角选择 REFERENCENORMALS 中最接近的一列。
%
%   输入
%     localNormal       机体系下的三维法向量
%     attitudeRad       3×1 ZYX 姿态 [roll; pitch; yaw] [rad]
%     referenceNormals  3×M 全局参考方向
%     toleranceRad      允许关联的最大夹角 [rad]
%
%   输出
%     referenceIndex    匹配的参考列索引；无法关联时返回 0
%     minimumAngleRad   最小有向法向夹角 [rad]
%     globalNormal      归一化后位于全局坐标系的输入法向量
%
%   法向量方向具有物理意义，因此 n 与 -n 被视为两个不同方向。
%
%   作者：Wenshao Lyu

    narginchk(4, 4);

    validateattributes(localNormal, {'double'}, ...
        {'vector', 'numel', 3, 'real'}, mfilename, 'localNormal', 1);
    validateattributes(attitudeRad, {'double'}, ...
        {'vector', 'numel', 3, 'real', 'finite'}, ...
        mfilename, 'attitudeRad', 2);
    validateattributes(referenceNormals, {'double'}, ...
        {'2d', 'real', 'finite', 'nonempty'}, ...
        mfilename, 'referenceNormals', 3);
    validateattributes(toleranceRad, {'double'}, ...
        {'scalar', 'real', 'finite', '>=', 0, '<=', pi}, ...
        mfilename, 'toleranceRad', 4);

    if size(referenceNormals, 1) ~= 3
        error('PlaneNormalAssociation:InvalidReferenceSize', ...
            'referenceNormals must have three rows.');
    end

    % 对退化或非有限观测直接返回“未关联”。
    [localNormal, isValidLocalNormal] = normalizeVectorSafe(localNormal);
    if ~isValidLocalNormal
        referenceIndex = 0;
        minimumAngleRad = inf;
        globalNormal = nan(3, 1);
        return;
    end

    referenceNorms = sqrt(sum(referenceNormals .^ 2, 1));
    if any(referenceNorms <= 1e-12)
        error('PlaneNormalAssociation:InvalidReferenceNormal', ...
            'Every reference normal must have nonzero magnitude.');
    end
    normalizedReferences = referenceNormals ./ referenceNorms;

    % 将局部法向量旋转至全局坐标系，再计算与参考方向的夹角。
    rotationBodyToGlobal = eulerZYXRotation(attitudeRad(:));
    globalNormal = rotationBodyToGlobal * localNormal;

    cosineAngles = globalNormal.' * normalizedReferences;
    cosineAngles = max(-1, min(1, cosineAngles));
    angularSeparationsRad = acos(cosineAngles);
    [minimumAngleRad, bestIndex] = min(angularSeparationsRad);

    % 仅在最近参考方向满足角度门限时接受关联结果。
    if minimumAngleRad <= toleranceRad
        referenceIndex = bestIndex;
    else
        referenceIndex = 0;
    end
end
