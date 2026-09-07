function collision = segment_aabb_collision(pA, pB, center, sideLength)
%SEGMENT_AABB_COLLISION 使用slab method检查线段与轴对齐立方体碰撞。
%   pA、pB和center均为3x1列向量，sideLength为立方体边长，单位为m。
%   AABB表示axis-aligned bounding box，其各边与base frame坐标轴平行。

    direction = pB - pA;
    halfSide = sideLength / 2;
    lower = center - halfSide;
    upper = center + halfSide;

    % t的初始范围[0,1]对应整条线段，而不是无限长直线。
    tMin = 0;
    tMax = 1;
    collision = true;

    for axis = 1:3
        if abs(direction(axis)) < 1e-12
            % 该轴坐标沿线段保持不变；若固定坐标在slab外，则不可能相交。
            if pA(axis) < lower(axis) || pA(axis) > upper(axis)
                collision = false;
                return;
            end
        else
            t1 = (lower(axis) - pA(axis)) / direction(axis);
            t2 = (upper(axis) - pA(axis)) / direction(axis);
            tEnter = min(t1, t2);
            tExit = max(t1, t2);

            % 逐轴求允许参数区间的交集。
            tMin = max(tMin, tEnter);
            tMax = min(tMax, tExit);

            if tMin > tMax
                collision = false;
                return;
            end
        end
    end
end
