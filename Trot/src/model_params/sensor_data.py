import mujoco_py
import numpy as np
import sys, os
import mujoco
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from LegModel.legs import LegModel
model = mujoco_py.load_model_from_path("../../models/dynamic_4l.xml")

sim = sim = mujoco_py.MjSim(model)
def print_sensor_info(model, sim=None):
    print("Sensor Information:")
    print("-------------------")
# 遍历所有传感器
    for i in range(model.nsensor):
        # 获取传感器名字
        sensor_name = model.sensor_names[i]
        
        # 获取传感器 ID（即索引，从 0 开始）
        sensor_id = i
        
        # 获取传感器类型
        sensor_type = model.sensor_type[i]
        
        # 获取传感器数据在 sensordata 中的起始索引
        sensor_adr = model.sensor_adr[i]
        
        # 获取传感器数据的长度
        sensor_dim = model.sensor_dim[i]
        
        # 如果提供了 sim 对象，打印当前 sensordata 值
        if sim is not None:
            sensor_data = sim.data.sensordata[sensor_adr:sensor_adr + sensor_dim]
            data_str = f", Data: {sensor_data}"
        else:
            data_str = ""
        
        # 打印信息
        print(f"Sensor ID: {sensor_id}, Name: {sensor_name}, Type: {sensor_type}, "
                f"Start Index: {sensor_adr}, Length: {sensor_dim}{data_str}")

print_sensor_info(model, sim)
