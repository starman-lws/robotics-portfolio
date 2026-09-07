function parameters = planar_pick_place_config()
%PLANAR_PICK_PLACE_CONFIG 返回通用平面拾放动作与RTDE参数。
%   TCP位置使用mm；blend radius使用m，以匹配当前RTDE接口。

    parameters.pickHeightMm = 6;
    parameters.travelOffsetMm = 6;

    parameters.jointAcceleration = 1;
    parameters.jointVelocity = 0.6;
    parameters.linearAcceleration = 1;
    parameters.linearVelocity = 0.6;
    parameters.time = 0;
    parameters.blendRadiusM = 0.001;

    parameters.motionPauseS = 0.1;
    parameters.gripPauseS = 0.5;
    parameters.releasePauseS = 0.5;

    parameters.homePose = [ ...
        -588.53, -133.30, 227.00, 2.221, 2.221, 0];
    parameters.positionToleranceMm = 1e-6;
    parameters.thetaToleranceRad = 1e-6;
    parameters.rotationMatrixTolerance = 1e-8;
end
