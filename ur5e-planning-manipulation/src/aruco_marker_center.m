function center = aruco_marker_center(corners)
%ARUCO_MARKER_CENTER 通过两条对角线的交点计算marker中心。
%   corners必须是4x2有限实数矩阵，每行为一个图像或平面坐标角点。
%   center为1x2行向量。该方法保持projective geometry下的对角线
%   交点，而不是直接对四个角点取算术平均。

    if ~isnumeric(corners) || ~isreal(corners) || ...
            ~isequal(size(corners), [4, 2]) || ...
            any(~isfinite(corners(:)))
        error('aruco_marker_center:InvalidCorners', ...
              'corners必须是4x2有限实数矩阵。');
    end

    homogeneousCorners = [double(corners), ones(4, 1)];
    line13 = cross(homogeneousCorners(1,:), ...
                   homogeneousCorners(3,:));
    line24 = cross(homogeneousCorners(2,:), ...
                   homogeneousCorners(4,:));
    homogeneousCenter = cross(line13, line24);

    scale = homogeneousCenter(3);
    scaleTolerance = 1e-12 * max(1, norm(homogeneousCenter));
    if ~isfinite(scale) || abs(scale) <= scaleTolerance
        error('aruco_marker_center:DegenerateDiagonals', ...
              'marker两条对角线没有有限且唯一的交点。');
    end

    center = homogeneousCenter(1:2) / scale;
end
