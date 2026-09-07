function scene = localize_pick_place_scene(image)
%LOCALIZE_PICK_PLACE_SCENE 从单张图像重建视觉引导拾放平面场景。
%   scene = LOCALIZE_PICK_PLACE_SCENE(image)检测DICT_ARUCO_ORIGINAL
%   markers，利用ID0-3建立图像到机器人Base平面的Homography，并返回
%   障碍物、小物体、大物体、目标板以及两个放置目标的二维位姿。
%
%   输入image可以是灰度或RGB数值/逻辑图像。输出位置和offset单位为mm，
%   theta单位为rad。该函数不访问webcam或机器人硬件。

    validate_image(image);
    config = pick_place_scene_config();

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
    bigMarkers = aruco_marker_pose_2d( ...
        imageToBaseTransform, corners, ids, 6);
    targetBoard = aruco_marker_pose_2d( ...
        imageToBaseTransform, corners, ids, 7);

    % 保持实机流程的聚合规则：中心取所有ID6中心平均值，朝向取第一个
    % ID6 marker。corners保留参与聚合的全部ID6 marker角点。
    bigItem.position = mean(bigMarkers.position, 1);
    bigItem.theta = bigMarkers.theta(1);
    bigItem.corners = bigMarkers.corners;

    smallGoal.position = targetBoard.position + ...
        transform_local_offset_2d( ...
            config.smallGoalOffset, targetBoard.theta);
    smallGoal.theta = targetBoard.theta;
    smallGoal.corners = targetBoard.corners;

    bigGoal.position = targetBoard.position + ...
        transform_local_offset_2d( ...
            config.bigGoalOffset, targetBoard.theta);
    bigGoal.theta = targetBoard.theta;
    bigGoal.corners = targetBoard.corners;

    scene.markerIDs = ids;
    scene.markerCorners = corners;
    scene.detectedFamily = detectedFamily;
    scene.imageToBaseTransform = imageToBaseTransform;
    scene.annotatedImage = annotatedImage;
    scene.boardImage = boardImage;
    scene.boardReference = boardReference;

    scene.obstacles = obstacles;
    scene.smallItem = smallItem;
    scene.bigMarkers = bigMarkers;
    scene.bigItem = bigItem;
    scene.targetBoard = targetBoard;
    scene.smallGoal = smallGoal;
    scene.bigGoal = bigGoal;

    % 保存reference点用于验证和后续README中的数值示例。
    scene.referenceImagePoints = referenceImagePoints;
    scene.referenceWorldPoints = config.referenceWorldPoints;
end

function validate_image(image)
    validChannelCount = size(image, 3) == 1 || size(image, 3) == 3;
    if ~(isnumeric(image) || islogical(image)) || ~isreal(image) || ...
            isempty(image) || ndims(image) > 3 || ~validChannelCount || ...
            any(~isfinite(image(:)))
        error('localize_pick_place_scene:InvalidImage', ...
              'image必须是非空、有限的二维灰度或三通道RGB图像。');
    end
end
