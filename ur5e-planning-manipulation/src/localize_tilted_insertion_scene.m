function scene = localize_tilted_insertion_scene(image)
%LOCALIZE_TILTED_INSERTION_SCENE 重建小物体倾斜插入视觉场景。
%   ID0-3建立图像到Z=0桌面的Homography；ID8角点随后利用已知相机
%   光心沿射线修正到Z=90 mm平面。函数只处理输入图像，不访问webcam
%   或机器人硬件。

    validate_image(image);
    config = tilted_insertion_scene_config();

    [ids, corners, detectedFamily] = ...
        readArucoMarker(image, config.markerFamily);
    selection = validate_aruco_detections(ids, corners, config);
    annotatedImage = annotate_aruco_markers( ...
        image, ids, corners, config);
    [imageToBaseTransform, referenceImagePoints] = ...
        estimate_board_homography(corners, selection, config);

    boardReference = imref2d(config.boardImageSize, ...
        config.boardXLimits, config.boardYLimits);
    [boardImage, boardReference] = imwarp(image, ...
        imageToBaseTransform, 'OutputView', boardReference);

    obstacles = aruco_marker_pose_2d( ...
        imageToBaseTransform, corners, ids, 4);
    smallItem = aruco_marker_pose_2d( ...
        imageToBaseTransform, corners, ids, 5);
    elevatedMarker = elevated_aruco_marker_pose( ...
        imageToBaseTransform, corners, ids, 8, ...
        config.cameraBasePosition, config.elevatedMarkerHeight);

    slotXY = elevatedMarker.position(1:2) ...
        + transform_local_offset_2d( ...
            config.slotOffset, elevatedMarker.theta);
    slotEntrance.position = [slotXY, config.slotEntranceHeight];
    slotEntrance.theta = elevatedMarker.theta;
    slotEntrance.corners = elevatedMarker.corners;

    scene.markerIDs = ids;
    scene.markerCorners = corners;
    scene.detectedFamily = detectedFamily;
    scene.imageToBaseTransform = imageToBaseTransform;
    scene.annotatedImage = annotatedImage;
    scene.boardImage = boardImage;
    scene.boardReference = boardReference;
    scene.referenceImagePoints = referenceImagePoints;
    scene.referenceWorldPoints = config.referenceWorldPoints;
    scene.cameraBasePosition = config.cameraBasePosition;

    scene.obstacles = obstacles;
    scene.smallItem = smallItem;
    scene.elevatedMarker = elevatedMarker;
    scene.slotEntrance = slotEntrance;
end

function validate_image(image)
    validChannelCount = size(image, 3) == 1 || size(image, 3) == 3;
    if ~(isnumeric(image) || islogical(image)) || ~isreal(image) ...
            || isempty(image) || ndims(image) > 3 || ~validChannelCount ...
            || any(~isfinite(image(:)))
        error('localize_tilted_insertion_scene:InvalidImage', ...
              'image必须是非空、有限的二维灰度或三通道RGB图像。');
    end
end
