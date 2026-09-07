function r = rotation_matrix_to_rotvec(R)
%ROTATION_MATRIX_TO_ROTVEC 将3x3旋转矩阵转换为1x3旋转向量。
%   旋转向量满足r=theta*u，其中theta为旋转角，u为单位旋转轴。

    % trace(R)=1+2*cos(theta)。限制c的范围可抵消浮点舍入误差。
    c = (trace(R) - 1) / 2;
    c = max(min(c, 1), -1);
    theta = acos(c);

    tolZero = 1e-10;
    tolPi = 1e-7;

    if abs(theta) < tolZero
        % 零旋转时旋转轴不唯一，rotation vector直接定义为零向量。
        r = [0, 0, 0];
        return;
    end

    if pi - theta < tolPi
        % theta接近pi时sin(theta)接近零，普通反对称公式数值不稳定。
        % 当theta=pi时，(R+I)/2=u*u^T，可由其最大对角元恢复旋转轴。
        A = (R + eye(3)) / 2;
        A = (A + A.') / 2;

        [~, k] = max(diag(A));
        u = zeros(3,1);
        u(k) = sqrt(max(A(k,k), 0));

        remaining = setdiff(1:3, k);
        u(remaining) = A(remaining,k) / u(k);
        u = u / norm(u);

        % u和-u在旋转pi时等价；非零反对称部分可用于保持符号连续性。
        skewVector = [R(3,2) - R(2,3);
                      R(1,3) - R(3,1);
                      R(2,1) - R(1,2)];
        if norm(skewVector) > tolZero && dot(u, skewVector) < 0
            u = -u;
        end
    else
        % 普通情况由R-R^T=2*sin(theta)*[u]x恢复单位旋转轴。
        u = [R(3,2) - R(2,3);
             R(1,3) - R(3,1);
             R(2,1) - R(1,2)] / (2*sin(theta));
        u = u / norm(u);
    end

    r = (theta * u).';
end
