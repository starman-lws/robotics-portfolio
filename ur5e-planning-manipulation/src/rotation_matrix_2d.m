function rotation = rotation_matrix_2d(theta)
%ROTATION_MATRIX_2D 构造逆时针旋转theta的二维旋转矩阵。
%   theta单位为rad。

    if ~isnumeric(theta) || ~isreal(theta) || ...
            ~isscalar(theta) || ~isfinite(theta)
        error('rotation_matrix_2d:InvalidAngle', ...
              'theta必须是有限实数标量。');
    end

    rotation = [cos(theta), -sin(theta);
                sin(theta),  cos(theta)];
end
