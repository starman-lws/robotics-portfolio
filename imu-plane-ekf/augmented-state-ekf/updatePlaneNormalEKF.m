function [posteriorState, posteriorCovariance, innovation] = ...
    updatePlaneNormalEKF(priorState, priorCovariance, ...
    measuredNormalBody, referenceNormalGlobal, measurementStd)
%UPDATEPLANENORMALEKF 使用有向平面法向量修正姿态状态。
%   观测模型预测全局参考法向量在机体系下的表达。更新前会分别
%   归一化测量法向量和参考法向量。
%
%   输入
%     priorState             6×1 先验姿态—偏置状态
%     priorCovariance        6×6 先验状态协方差
%     measuredNormalBody     3×1 机体系下的测量法向量
%     referenceNormalGlobal  3×1 已关联的全局参考法向量
%     measurementStd         法向量各分量的标量标准差
%
%   输出
%     posteriorState         6×1 后验修正状态
%     posteriorCovariance    采用 Joseph form 更新的后验协方差
%     innovation             测量法向量与预测法向量之差
%
%   法向量方向具有物理意义，更新时不会将 n 与 -n 视为等价。
%
%   作者：Wenshao Lyu

    narginchk(5, 5);

    validateattributes(priorState, {'double'}, ...
        {'vector', 'numel', 6, 'real', 'finite'}, ...
        mfilename, 'priorState', 1);
    validateattributes(priorCovariance, {'double'}, ...
        {'size', [6, 6], 'real', 'finite'}, ...
        mfilename, 'priorCovariance', 2);
    validateattributes(measuredNormalBody, {'double'}, ...
        {'vector', 'numel', 3, 'real'}, ...
        mfilename, 'measuredNormalBody', 3);
    validateattributes(referenceNormalGlobal, {'double'}, ...
        {'vector', 'numel', 3, 'real'}, ...
        mfilename, 'referenceNormalGlobal', 4);
    validateattributes(measurementStd, {'double'}, ...
        {'scalar', 'real', 'finite', 'positive'}, ...
        mfilename, 'measurementStd', 5);

    % 归一化观测与参考方向，并拒绝退化法向量。
    [measuredNormalBody, isValidMeasurement] = ...
        normalizeVectorSafe(measuredNormalBody);
    [referenceNormalGlobal, isValidReference] = ...
        normalizeVectorSafe(referenceNormalGlobal);
    if ~isValidMeasurement || ~isValidReference
        error('AttitudeBiasEKF:InvalidPlaneNormal', ...
            'Measured and reference normals must be finite and nonzero.');
    end

    priorState = priorState(:);
    % 计算非线性法向量观测及其解析 Jacobian。
    [predictedNormalBody, measurementJacobian] = ...
        planeNormalMeasurementModel( ...
        priorState(1:3), referenceNormalGlobal);
    innovation = measuredNormalBody - predictedNormalBody;
    measurementCovariance = (measurementStd ^ 2) * eye(3);

    % 使用 Joseph form 保持协方差的数值稳定性。
    [posteriorState, posteriorCovariance] = josephCovarianceUpdate( ...
        priorState, priorCovariance, innovation, ...
        measurementJacobian, measurementCovariance);
end
