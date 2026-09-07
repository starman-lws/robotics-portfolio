function parameters = tilted_insertion_motion_config()
%TILTED_INSERTION_MOTION_CONFIG 返回倾斜插入动作与RTDE参数。
%   TCP位置单位为mm，角度单位为rad，blend radius单位为m。

    parameters.pickHeightMm = 6;
    parameters.travelOffsetMm = 150;
    parameters.tiltAngleRad = deg2rad(45);
    parameters.tiltClearanceOffsetMm = -40;
    parameters.insertionDistanceMm = 50;
    parameters.retreatDistanceMm = -50;

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
end
