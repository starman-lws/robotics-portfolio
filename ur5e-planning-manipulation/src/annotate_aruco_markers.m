function annotatedImage = annotate_aruco_markers(image, ids, corners, config)
%ANNOTATE_ARUCO_MARKERS 在图像中标出marker边框、角点、中心和ID。

    annotatedImage = image;
    markerCount = numel(ids);

    for markerIndex = 1:markerCount
        markerCorners = corners(:, :, markerIndex);
        annotatedImage = insertShape(annotatedImage, 'polygon', ...
            {markerCorners}, Opacity=1, ShapeColor='green', ...
            LineWidth=config.markerEdgeWidth);

        cornerPositions = [markerCorners, ...
            repmat(config.markerCornerRadius, 4, 1)];
        annotatedImage = insertShape(annotatedImage, 'FilledCircle', ...
            cornerPositions, ShapeColor='red', Opacity=1);

        markerCenter = aruco_marker_center(markerCorners);
        annotatedImage = insertText(annotatedImage, markerCenter, ...
            ids(markerIndex), FontSize=config.markerLabelFontSize, ...
            BoxOpacity=1);
    end
end
