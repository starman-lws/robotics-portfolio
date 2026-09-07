function transformedPoints = transformPointCloudZYX(points, attitudeRad, translation)
%transformPointCloudZYX 使用 ZYX Euler 姿态与平移变换三维点云
%   transformedPoints = transformPointCloudZYX(points, attitudeRad, translation)
%   对每个输入点执行 p_target = Rz(yaw)*Ry(pitch)*Rx(roll)*p_source + t。
%
%   输入：
%     points       - N×3 点云，每行一个三维点，建议使用单位 m
%     attitudeRad  - [roll; pitch; yaw]，单位为 rad，可为行或列向量
%     translation  - 3 元平移向量，单位应与 points 一致
%
%   输出：
%     transformedPoints - N×3 目标坐标系点云
%
%   作者：Wenshao Lyu

    arguments
        points (:,:) double {mustBeReal, mustBeFinite, mustBeNonempty}
        attitudeRad double {mustBeReal, mustBeFinite}
        translation double {mustBeReal, mustBeFinite}
    end

    if size(points, 2) ~= 3
        error('PlaneEstimation:InvalidPointShape', ...
            'points must be an N-by-3 matrix.');
    end
    if numel(attitudeRad) ~= 3
        error('PlaneEstimation:InvalidAttitudeSize', ...
            'attitudeRad must contain exactly three elements.');
    end
    if numel(translation) ~= 3
        error('PlaneEstimation:InvalidTranslationSize', ...
            'translation must contain exactly three elements.');
    end

    rotation = eulerZYXRotation(attitudeRad(:));
    transformedPoints = (rotation * points.').'+ translation(:).';
end
