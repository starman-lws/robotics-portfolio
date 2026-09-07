function config = tilted_insertion_scene_config()
%TILTED_INSERTION_SCENE_CONFIG 返回倾斜插入场景的固定视觉参数。
%   Base位置和高度的单位为mm。ID8位于与桌面平行的已知高度平面。

    config.markerFamily = "DICT_ARUCO_ORIGINAL";

    config.referenceIDs = [1, 3, 0, 2];
    config.referenceWorldPoints = [ ...
        -230,   60; ...
        -230, -520; ...
        -990,   60; ...
        -990, -520];

    config.requiredIDs = [0, 1, 2, 3, 4, 5, 8];
    config.repeatableIDs = 4;
    config.singleInstanceIDs = [5, 8];

    config.boardImageSize = [680, 860];
    config.boardXLimits = [-1040, -180];
    config.boardYLimits = [-570, 110];

    config.cameraBasePosition = [-1300, -60, 860];
    config.elevatedMarkerHeight = 90;
    config.slotOffset = [0; -80];
    config.slotEntranceHeight = 60;

    config.markerCornerRadius = 6;
    config.markerEdgeWidth = 4;
    config.markerLabelFontSize = 30;
end
