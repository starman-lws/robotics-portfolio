function covariance = symmetrizeCovariance(covariance)
%SYMMETRIZECOVARIANCE 消除浮点运算引入的协方差非对称误差。
%
%   作者：Wenshao Lyu

    covariance = 0.5 * (covariance + covariance.');
end
