function [T05, eePose] = ur5e_locked_elbow_forward_kinematics(q)
%UR5E_LOCKED_ELBOW_FORWARD_KINEMATICS 计算等效五自由度模型的末端位姿。
%   q      : 1x6或6x1关节角向量，单位为degree。
%            第三个关节必须固定在90 degree。
%   T05    : 等效第五个frame相对于base frame的4x4齐次变换矩阵。
%   eePose : [x,y,z,rx,ry,rz]，位置单位为mm，旋转向量单位为rad。

    dhTable = ur5e_locked_elbow_dh(q);

    T05 = eye(4);
    for linkIndex = 1:size(dhTable, 1)
        TRelative = dh_transform(dhTable(linkIndex,1), ...
                                 dhTable(linkIndex,2), ...
                                 dhTable(linkIndex,3), ...
                                 dhTable(linkIndex,4));
        T05 = T05 * TRelative;
    end

    positionMm = 1000 * T05(1:3,4).';
    rotationVector = rotation_matrix_to_rotvec(T05(1:3,1:3));
    eePose = [positionMm, rotationVector];
end
