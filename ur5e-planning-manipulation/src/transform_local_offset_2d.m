function offsetBase = transform_local_offset_2d(offsetLocal, theta)
%TRANSFORM_LOCAL_OFFSET_2D 将marker局部offset旋转到Base坐标系。
%   offsetLocal可以是1x2或2x1有限实数向量，输出为1x2行向量。

    if ~isnumeric(offsetLocal) || ~isreal(offsetLocal) || ...
            ~isvector(offsetLocal) || numel(offsetLocal) ~= 2 || ...
            any(~isfinite(offsetLocal(:)))
        error('transform_local_offset_2d:InvalidOffset', ...
              'offsetLocal必须是包含两个有限实数的向量。');
    end

    offsetBase = (rotation_matrix_2d(theta) * offsetLocal(:)).';
end
