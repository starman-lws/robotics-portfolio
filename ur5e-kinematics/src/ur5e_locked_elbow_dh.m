function dhTable = ur5e_locked_elbow_dh(q)
%UR5E_LOCKED_ELBOW_DH 构造肘关节固定后的等效五自由度DH表。
%   q       : 1x6或6x1关节角向量，单位为degree。
%             第三个关节必须固定在90 degree。
%   dhTable : 5x4标准DH参数矩阵，每行为[theta,d,a,alpha]。
%             角度单位为rad，长度单位为m。

    if ~isnumeric(q) || ~isreal(q) || ~isvector(q) || ...
            numel(q) ~= 6 || any(~isfinite(q(:)))
        error('ur5e_locked_elbow_dh:InvalidJointVector', ...
              'q必须是包含六个有限实数的关节角向量。');
    end

    q = q(:).';

    lockedAngleDeg = 90;
    lockedAngleToleranceDeg = 1e-6;
    if abs(q(3) - lockedAngleDeg) > lockedAngleToleranceDeg
        error('ur5e_locked_elbow_dh:ElbowNotLocked', ...
              '第三个关节必须固定在90 degree。');
    end

    qRad = deg2rad(q);

    % 固定的第三关节使原第二、第三连杆在同一平面内形成合成位移。
    aEquivalent = -hypot(0.425, 0.3922);
    beta = atan2(0.3922, 0.425);
    delta = pi/2 - beta;

    % 原六关节变量映射为[q1,q2,q4,q5,q6]，并吸收两个固定角偏移。
    thetaEquivalent = [qRad(1), ...
                       qRad(2) + beta, ...
                       qRad(4) + delta, ...
                       qRad(5), ...
                       qRad(6)];

    dhTable = [thetaEquivalent(1), 0.1625,           0,  pi/2;
               thetaEquivalent(2),      0, aEquivalent,     0;
               thetaEquivalent(3), 0.1333,           0,  pi/2;
               thetaEquivalent(4), 0.0997,           0, -pi/2;
               thetaEquivalent(5), 0.0996,           0,      0];
end
