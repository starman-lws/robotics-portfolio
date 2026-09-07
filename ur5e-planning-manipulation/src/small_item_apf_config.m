function parameters = small_item_apf_config()
%SMALL_ITEM_APF_CONFIG 返回小物体人工势场规划参数。
%   距离和步长单位均为mm。

    parameters.gridSpacing = 20;
    parameters.step = 5;
    parameters.maximumIterations = 1000;
    parameters.smoothingFactor = 0.4;

    parameters.attractionSwitchDistance = 190;
    parameters.attractiveGain = 2;

    parameters.repulsionInfluenceDistance = 200;
    parameters.repulsiveGain = 9e8;
    parameters.maximumRepulsion = 550;

    parameters.forceTolerance = 1e-9;
end
