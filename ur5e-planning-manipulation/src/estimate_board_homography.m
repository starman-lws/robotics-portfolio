function [transform, referenceImagePoints] = ...
        estimate_board_homography(corners, selection, config)
%ESTIMATE_BOARD_HOMOGRAPHY 估计图像坐标到Base平面坐标的projective变换。
%   referenceImagePoints按config.referenceIDs的顺序排列，中心由marker
%   两条对角线的交点计算。

    referenceCount = numel(config.referenceIDs);
    referenceImagePoints = zeros(referenceCount, 2);

    for referenceIndex = 1:referenceCount
        markerIndex = selection.referenceIndices(referenceIndex);
        referenceImagePoints(referenceIndex, :) = ...
            aruco_marker_center(corners(:, :, markerIndex));
    end

    try
        transform = fitgeotrans(referenceImagePoints, ...
            config.referenceWorldPoints, 'projective');
    catch exception
        wrappedException = MException( ...
            'estimate_board_homography:EstimationFailed', ...
            '无法根据ID0-3估计图像到Base坐标的Homography。');
        wrappedException = addCause(wrappedException, exception);
        throw(wrappedException);
    end
end
