function [eePose, ok] = ur5e_forward_kinematics(q, objects)
%UR5E_FORWARD_KINEMATICS 计算UR5e末端位姿并检查配置有效性。
%   q       : 1x6或6x1关节角，单位为degree。
%   objects : Nx5障碍物矩阵，每行为[x,y,z,type,size]，单位为m。
%             type=1表示球体，size为半径；type=2表示轴对齐立方体，
%             size为边长。
%   eePose  : [x,y,z,rx,ry,rz]，位置单位为mm，旋转向量单位为rad。
%   ok      : 配置有效时为1，否则为0。

    if length(q) ~= 6
        error('Must enter six joint angles!');
    end

    % 标准DH参数顺序为[theta,d,a,alpha]，角度使用rad，长度使用m。
    qRad = deg2rad(q);
    dhTable = [qRad(1), 0.1625,  0,       pi/2;
               qRad(2), 0,      -0.425,  0;
               qRad(3), 0,      -0.3922, 0;
               qRad(4), 0.1333,  0,       pi/2;
               qRad(5), 0.0997,  0,      -pi/2;
               qRad(6), 0.0996,  0,       0];

    % frameOrigins的各列依次为[p0,p1,...,p6]，全部在base frame中表达。
    frameOrigins = zeros(3, 7);
    T06 = eye(4);

    for i = 1:6
        TRelative = dh_transform(dhTable(i,1), dhTable(i,2), ...
                                 dhTable(i,3), dhTable(i,4));
        T06 = T06 * TRelative;
        frameOrigins(:,i+1) = T06(1:3,4);
    end

    % 从T06提取末端位置和旋转；内部位置为m，输出时转换为mm。
    positionMm = 1000 * T06(1:3,4).';
    rotationVector = rotation_matrix_to_rotvec(T06(1:3,1:3));
    eePose = [positionMm, rotationVector];

    % 安全平面有效性检查覆盖p1到p6；位于边界的base origin p0不参与检查。
    ok = check_workspace(frameOrigins(:,2:end));
    if ~ok
        ok = 0;
        eePose = zeros(1,6);
        return;
    end

    % 将相邻frame origins之间的机械臂部分近似为零厚度线段。
    for segmentIndex = 1:size(frameOrigins,2)-1
        pA = frameOrigins(:,segmentIndex);
        pB = frameOrigins(:,segmentIndex+1);

        for objectIndex = 1:size(objects,1)
            object = objects(objectIndex,:);
            center = object(1:3).';
            objectType = object(4);
            objectSize = object(5);

            if objectType == 1
                collision = segment_sphere_collision( ...
                    pA, pB, center, objectSize);
            elseif objectType == 2
                collision = segment_aabb_collision( ...
                    pA, pB, center, objectSize);
            else
                % 当前仅处理球体和轴对齐立方体，其他障碍物类型直接跳过。
                continue;
            end

            if collision
                ok = 0;
                eePose = zeros(1,6);
                return;
            end
        end
    end

    ok = 1;
end
