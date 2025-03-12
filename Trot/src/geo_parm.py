# robot_params.py
# 机器人参数文件，定义所有几何体的尺寸、质量和相对位置

robot_params = {
    "mouse": {  #老鼠身体为base，抽象为box
        "type": "box",
        "size": (0.068, 0.036, 0.031),  # 长、宽、高
        "mass": 0.15,
        "pos": (0, 0, 0.1)  # 世界坐标中的位置

    },
    "head": {  # bracket + mouse_head = head(0.001+0.01)
        "type": "box",
        "size": (0.3, 0.2, 0.4),
        "mass": 0.011,
        "pos": (0, 0, 0.5)
    },
    "fl": {  # 左前腿
        "type": "point",
        "pos": (-0.25, 0, 0.6),

    },
    "fr": {  # 右前腿
        "type": "point",
        "pos": (0.25, 0, 0.6)
    },
    "rl": {  # 左后腿
        "type": "capsule",
        "size": (0.06, 0.4),
        "mass": 5.0,
        "pos": (-0.15, 0, 0.2)
    },
    "rr": {  # 右后腿
        "type": "capsule",
        "size": (0.06, 0.4),
        "mass": 5.0,
        "pos": (0.15, 0, 0.2)
    },
    "sensors": {  # 传感器（假设是一个点）
        "type": "point",
        "size": (0.01,),
        "mass": 0.1,
        "pos": (0, 0.2, 0.8)
    }
}
