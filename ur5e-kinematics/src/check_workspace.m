function ok = check_workspace(frameOrigins)
%CHECK_WORKSPACE 检查p1到p6是否严格位于三个安全平面的有效侧。
%   frameOrigins为3x6矩阵，每列是一个在base frame中表达的原点。
%   平面表示为[nx,ny,nz,d]，合法条件为n^T*p+d>0。

    planes = [ 0,  0, 1,   0;   % Table: z > 0
               0, -1, 0, 0.3;   % Back:  y < 0.3 m
              -1,  0, 0, 0.3];  % Side:  x < 0.3 m

    ok = 1;

    for pointIndex = 1:size(frameOrigins,2)
        point = frameOrigins(:,pointIndex);

        for planeIndex = 1:size(planes,1)
            normal = planes(planeIndex,1:3);
            distance = planes(planeIndex,4);

            % 等于零表示点位于安全平面上；本模型将边界视为无效区域。
            if dot(normal, point.') + distance <= 0
                ok = 0;
                return;
            end
        end
    end
end
