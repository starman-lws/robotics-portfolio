function corners = large_item_corners(pose, localCorners)
%LARGE_ITEM_CORNERS 计算180x90 mm大物体在Base平面中的四个角点。
%   pose为[x,y,theta]。localCorners可省略，默认使用项目几何配置。

    if nargin < 2
        config = large_item_planning_config();
        localCorners = config.geometry.localCorners;
    end

    if ~isnumeric(pose) || ~isreal(pose) ...
            || ~isequal(size(pose), [1, 3]) ...
            || any(~isfinite(pose(:)))
        error('large_item_corners:InvalidPose', ...
              'pose必须是1x3有限实数向量。');
    end
    if ~isnumeric(localCorners) || ~isreal(localCorners) ...
            || ~isequal(size(localCorners), [4, 2]) ...
            || any(~isfinite(localCorners(:)))
        error('large_item_corners:InvalidGeometry', ...
              'localCorners必须是4x2有限实数矩阵。');
    end

    theta = double(pose(3));
    rotation = [cos(theta), -sin(theta); ...
                sin(theta),  cos(theta)];
    corners = double(pose(1:2)) ...
        + (rotation * double(localCorners).').';
end
