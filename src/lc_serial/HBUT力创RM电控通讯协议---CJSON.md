# HBUT力创RM电控通讯协议---CJSON

### 1.Json打包格式

通讯采用CJSON打包，打包格式如下

```json
{
    "cmd": "ctr_mode",
    "dat": {
        "date": [send_yaw, send_pitch, send_is_fire],
        "mode": "visual"
    }
}
```

哨兵机器人在原有云台控制基础上追加速度

```json
{
    "cmd": "ctr_mode",
    "dat": {
        "date": [send_yaw, send_pitch, send_is_fire, v_x, v_y],
        "mode": "visual"
    }
}
```

### 2.控制坐标系说明

对于烧饼大小yaw，控制消息中的vx, vy坐标系为大yaw正方向。