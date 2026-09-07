function factors = ur5e_singularity_factors(q)
%UR5E_SINGULARITY_FACTORS 计算UR5e的解析奇异因子。
%   q       : 1x6或6x1关节角向量，单位为degree。
%   factors : 包含shoulder、elbow和wrist三个signed因子的结构体。
%             shoulder单位为m，elbow和wrist为无量纲量。

    if ~isnumeric(q) || ~isreal(q) || ~isvector(q) || ...
            numel(q) ~= 6 || any(~isfinite(q(:)))
        error('ur5e_singularity_factors:InvalidJointVector', ...
              'q必须是包含六个有限实数的关节角向量。');
    end

    qRad = deg2rad(q(:).');

    a2 = -0.425;
    a3 = -0.3922;
    d5 = 0.0997;

    % 各因子为零时分别落在对应的解析奇异曲面上。
    factors.shoulder = a2*cos(qRad(2)) ...
                     + a3*cos(qRad(2) + qRad(3)) ...
                     + d5*sin(qRad(2) + qRad(3) + qRad(4));
    factors.elbow = sin(qRad(3));
    factors.wrist = sin(qRad(5));
end
