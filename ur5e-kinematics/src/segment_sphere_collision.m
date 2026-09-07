function collision = segment_sphere_collision(pA, pB, center, radius)
%SEGMENT_SPHERE_COLLISION 判断三维线段是否进入或接触球体。
%   pA、pB和center均为3x1列向量，坐标单位为m。

    direction = pB - pA;

    % 先求球心在线段所在直线上的投影参数，再限制到线段范围[0,1]。
    t = dot(center - pA, direction) / dot(direction, direction);
    t = max(0, min(1, t));
    closestPoint = pA + t*direction;

    % 距离等于半径表示线段与球体相切，同样判定为碰撞。
    collision = norm(closestPoint - center) <= radius;
end
