function T = dh_transform(theta, d, a, alpha)
%DH_TRANSFORM 根据一行标准DH参数构造齐次变换矩阵。
%   theta、alpha的单位为rad，d、a的单位为m。
%   T将当前frame中的齐次坐标转换到前一个frame中。

    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
