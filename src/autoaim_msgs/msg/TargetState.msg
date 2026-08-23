float32 x          # 世界系旋转中心位置 x (m)
float32 y          # 世界系旋转中心位置 y (m)
float32 z          # 世界系旋转中心位置 z (m)
float32 vx         # 世界系速度 x (m/s)
float32 vy         # 世界系速度 y (m/s)
float32 vz         # 世界系速度 z (m/s)
float32 yaw        # 目标朝向 (rad)
float32 w          # yaw 角速度 (rad/s)
float32 r          # 旋转半径 (m)
float32 armor_x    # 当前可见装甲板世界系位置（PnP 直接解算值）
float32 armor_y
float32 armor_z
int8 state         # 0=lost 1=detecting 2=tracking 3=temp_lost 4=switching
uint32 frame       # 配对帧号（与日志/截图对齐）
float32 gimbal_yaw # 云台当前 yaw (rad)，运动云台下目标稳定性的对照量
float32 gimbal_pitch
