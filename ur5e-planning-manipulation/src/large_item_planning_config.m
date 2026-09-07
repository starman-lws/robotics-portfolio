function config = large_item_planning_config()
%LARGE_ITEM_PLANNING_CONFIG 返回大物体APF、A*和几何诊断参数。
%   距离单位为mm，平面角度单位为rad。

    geometry.localCorners = [ ...
        -90, -45; 90, -45; -90, 45; 90, 45];
    geometry.cornerRadius = hypot(90, 45);
    geometry.minimumCenterDistance = 100;
    geometry.minimumCornerDistance = 70;
    geometry.collisionXYResolution = 2.5;
    geometry.collisionThetaResolution = deg2rad(1.5);
    geometry.numericalTolerance = 1e-10;

    apf.step = 5;
    apf.maximumIterations = 5000;
    apf.maximumThetaStep = deg2rad(3);
    apf.torqueTolerance = 1e-6;
    apf.positionTolerance = 5;
    apf.thetaStartDistance = 100;
    apf.goalTorqueGain = 2e4;
    apf.torqueToAngleGain = 5e-6;
    apf.translationFilterWeight = 0.5;
    apf.rotationFilterWeight = 0.5;
    apf.attractionSwitchDistance = 200;
    apf.attractiveGain = 2;
    apf.centerRepulsionDistance = 200;
    apf.centerRepulsiveGain = 1e8;
    apf.centerMaximumRepulsion = 600;
    apf.cornerRepulsionDistance = 80;
    apf.cornerRepulsiveGain = 1e9;
    apf.cornerMaximumRepulsion = 600;
    apf.forceTolerance = 1e-9;
    apf.motionTolerance = 1e-9;

    astar.xyResolution = 5;
    astar.thetaResolution = deg2rad(3);
    astar.transitionXYResolution = geometry.collisionXYResolution;
    astar.transitionThetaResolution = ...
        geometry.collisionThetaResolution;

    smoothing.outputXYResolution = 5;
    smoothing.outputThetaResolution = deg2rad(3);
    smoothing.roundingFraction = 0.35;
    smoothing.maximumRoundingTrim = 30;
    smoothing.maximumRoundingAttempts = 6;

    config.geometry = geometry;
    config.apf = apf;
    config.astar = astar;
    config.smoothing = smoothing;
end
