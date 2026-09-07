function config = pick_place_scene_config()
%PICK_PLACE_SCENE_CONFIG 返回视觉引导拾放场景使用的固定定位参数。
%   所有Base平面位置和offset的单位均为mm。

    config.markerFamily = "DICT_ARUCO_ORIGINAL";

    % referenceIDs的顺序与referenceWorldPoints的行一一对应。
    config.referenceIDs = [1, 3, 0, 2];
    config.referenceWorldPoints = [
        -230,   60;
        -230, -520;
        -990,   60;
        -990, -520
    ];

    config.requiredIDs = 0:7;
    config.repeatableIDs = [4, 6];
    config.singleInstanceIDs = [5, 7];

    config.boardImageSize = [680, 860];
    config.boardXLimits = [-1040, -180];
    config.boardYLimits = [-570, 110];

    config.smallGoalOffset = [-80; 0];
    config.bigGoalOffset = [-40; -100];

    config.markerCornerRadius = 6;
    config.markerEdgeWidth = 4;
    config.markerLabelFontSize = 30;
end
