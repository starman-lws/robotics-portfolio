function [posteriorState, posteriorCovariance] = ...
    josephCovarianceUpdate(priorState, priorCovariance, innovation, ...
    measurementJacobian, measurementCovariance)
%JOSEPHCOVARIANCEUPDATE 执行数值稳定的 Kalman 状态与协方差修正。
%
%   作者：Wenshao Lyu

    % 计算新息协方差和 Kalman 增益。
    innovationCovariance = measurementJacobian * priorCovariance * ...
        measurementJacobian.' + measurementCovariance;
    kalmanGain = priorCovariance * measurementJacobian.' / ...
        innovationCovariance;

    % 使用 Joseph form 更新协方差，降低舍入误差造成的半正定性损失。
    posteriorState = priorState + kalmanGain * innovation;
    identityMatrix = eye(size(priorCovariance));
    covarianceFactor = identityMatrix - kalmanGain * measurementJacobian;
    posteriorCovariance = covarianceFactor * priorCovariance * ...
        covarianceFactor.' + kalmanGain * measurementCovariance * ...
        kalmanGain.';
    posteriorCovariance = symmetrizeCovariance(posteriorCovariance);
end
