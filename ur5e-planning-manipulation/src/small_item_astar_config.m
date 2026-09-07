function config = small_item_astar_config()
%SMALL_ITEM_ASTAR_CONFIG 返回小物体二维A*、平滑和几何参数。
%   距离单位均为mm。

    geometry.itemRadius = 65;
    geometry.obstacleRadius = 50;
    geometry.minimumCenterDistance = 115;
    geometry.collisionResolution = 2.5;
    geometry.numericalTolerance = 1e-10;

    algorithm.xyResolution = 5;
    algorithm.transitionResolution = geometry.collisionResolution;

    smoothing.outputResolution = 5;
    smoothing.roundingFraction = 0.35;
    smoothing.maximumRoundingTrim = 30;
    smoothing.maximumRoundingAttempts = 6;

    config.geometry = geometry;
    config.algorithm = algorithm;
    config.smoothing = smoothing;
end
