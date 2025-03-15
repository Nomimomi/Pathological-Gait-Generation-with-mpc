import mujoco_py
import os

# 设置 XML 文件路径
xml_path = "../models/tail_assets/tail.xml"  # 替换为实际的文件路径

# 加载 MuJoCo 模型
model = mujoco_py.load_model_from_path(xml_path)

# 创建仿真对象
sim = mujoco_py.MjSim(model)

# 创建渲染器
viewer = mujoco_py.MjViewer(sim)

# 渲染并模拟
while True:
    sim.step()  # 执行一步仿真
    viewer.render()  # 渲染当前仿真状态