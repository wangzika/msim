# ADR-0003：坐标与外参

状态：草案。earth 为 ECEF，map 为场景原点 ENU，base_link 为前左上；T_parent_child 将 child 坐标转换到 parent，四元数以 x,y,z,w 存储。天线和 IMU 位置从刚体状态与外参派生。具体校验及数值验收待 P1。
