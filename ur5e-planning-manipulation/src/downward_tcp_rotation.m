function rotationVector = downward_tcp_rotation(theta)
%DOWNWARD_TCP_ROTATION 将Base平面朝向转换为吸盘向下的旋转向量。
%   theta为绕Base Z轴的平面朝向，单位为rad。输出为UR5e使用的1x3
%   axis-angle rotation vector。

    if ~isnumeric(theta) || ~isreal(theta) || ~isscalar(theta) ...
            || ~isfinite(theta)
        error('downward_tcp_rotation:InvalidTheta', ...
              'theta必须是有限实数标量。');
    end

    theta = atan2(sin(double(theta)), cos(double(theta)));
    rotationX = [1, 0, 0; 0, -1, 0; 0, 0, -1];
    rotationZ = [cos(theta), -sin(theta), 0; ...
                 sin(theta),  cos(theta), 0; ...
                 0,           0,          1];

    rotationVector = rotmat2vec3d(rotationZ * rotationX);
end
