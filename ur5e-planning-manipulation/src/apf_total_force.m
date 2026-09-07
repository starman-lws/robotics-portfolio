function force = apf_total_force(position, goal, obstacles, parameters)
%APF_TOTAL_FORCE 计算目标吸引力与所有障碍物排斥力之和。

    force = apf_attractive_force(position, goal, parameters) ...
          + apf_repulsive_force(position, obstacles, parameters);
end
