function rotation = eulerZYXRotation(attitudeRad)
%eulerZYXRotation 根据 [roll; pitch; yaw] 构造 ZYX 旋转矩阵
%   输入角度单位为 rad，输出满足 R = Rz(yaw)*Ry(pitch)*Rx(roll)。
%
%   作者：Wenshao Lyu

    roll = attitudeRad(1);
    pitch = attitudeRad(2);
    yaw = attitudeRad(3);

    cosRoll = cos(roll);
    sinRoll = sin(roll);
    cosPitch = cos(pitch);
    sinPitch = sin(pitch);
    cosYaw = cos(yaw);
    sinYaw = sin(yaw);

    rotationX = [1, 0, 0; ...
                 0, cosRoll, -sinRoll; ...
                 0, sinRoll, cosRoll];
    rotationY = [cosPitch, 0, sinPitch; ...
                 0, 1, 0; ...
                 -sinPitch, 0, cosPitch];
    rotationZ = [cosYaw, -sinYaw, 0; ...
                 sinYaw, cosYaw, 0; ...
                 0, 0, 1];

    rotation = rotationZ * rotationY * rotationX;
end
