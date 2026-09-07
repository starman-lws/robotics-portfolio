function [J, Jv, Jw] = ur5e_geometric_jacobian(q)
%UR5E_GEOMETRIC_JACOBIAN 计算UR5e的几何Jacobian。
%   q  : 1x6或6x1关节角向量，单位为degree。
%   J  : 6x6几何Jacobian，行顺序为[vx,vy,vz,wx,wy,wz]。
%   Jv : 3x6线速度映射，长度单位为m。
%   Jw : 3x6角速度映射。

    if ~isnumeric(q) || ~isreal(q) || ~isvector(q) || ...
            numel(q) ~= 6 || any(~isfinite(q(:)))
        error('ur5e_geometric_jacobian:InvalidJointVector', ...
              'q必须是包含六个有限实数的关节角向量。');
    end

    qRad = deg2rad(q(:).');

    % 标准DH参数顺序为[theta,d,a,alpha]，角度使用rad，长度使用m。
    dhTable = [qRad(1), 0.1625,  0,       pi/2;
               qRad(2), 0,      -0.425,  0;
               qRad(3), 0,      -0.3922, 0;
               qRad(4), 0.1333,  0,       pi/2;
               qRad(5), 0.0997,  0,      -pi/2;
               qRad(6), 0.0996,  0,       0];

    % origins和jointAxes均在base frame中表达。
    % 第i列分别对应o_(i-1)和z_(i-1)，用于构造第i个关节列。
    origins = zeros(3, 7);
    jointAxes = zeros(3, 6);
    TCurrent = eye(4);

    for jointIndex = 1:6
        jointAxes(:,jointIndex) = TCurrent(1:3,3);

        TRelative = dh_transform(dhTable(jointIndex,1), ...
                                 dhTable(jointIndex,2), ...
                                 dhTable(jointIndex,3), ...
                                 dhTable(jointIndex,4));
        TCurrent = TCurrent * TRelative;
        origins(:,jointIndex+1) = TCurrent(1:3,4);
    end

    endEffectorOrigin = origins(:,7);
    Jv = zeros(3, 6);
    Jw = jointAxes;

    % UR5e的六个关节均为转动关节。
    for jointIndex = 1:6
        Jv(:,jointIndex) = cross( ...
            jointAxes(:,jointIndex), ...
            endEffectorOrigin - origins(:,jointIndex));
    end

    J = [Jv; Jw];
end
