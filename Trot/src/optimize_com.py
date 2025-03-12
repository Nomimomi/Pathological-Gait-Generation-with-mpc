import numpy as np
import matplotlib.pyplot as plt
import mujoco_py
from ToSim import SimModel
from Controller import MouseController
import center_of_mass


def load_model():
    model = mujoco_py.load_model_from_path("../models/dynamic_4l.xml")
    sim = mujoco_py.MjSim(model)
    return model, sim

def create_viewer(sim):
    viewer = mujoco_py.MjViewer(sim)
    return viewer

def run_simulation(sim, length, theController, theMouse, run_steps_num, time_step, ignore_steps=500):
    initial_com = center_of_mass.get_com_ref(theMouse.model, theMouse.sim, "mouse_bracket", 0)
    difference_x, difference_y, difference_z = [], [], []
    pre_com = initial_com

    for i in range(run_steps_num):
        if i < ignore_steps:
            continue

        tCtrlData = theController.runStep()
        ctrlData = tCtrlData
        theMouse.runStep(ctrlData, time_step, render_flag=True)

        curr_com = center_of_mass.get_com_ref(theMouse.model, theMouse.sim, "mouse_bracket", 0)
        com_eval = center_of_mass.get_com(theMouse.model, theMouse.sim, 0, length)

        difference_x.append(com_eval[0] - curr_com[0])
        difference_y.append(com_eval[1] - curr_com[1])
        difference_z.append(com_eval[2] - curr_com[2])

        pre_com = curr_com

    total_error = np.sum(np.square(difference_x)) + np.sum(np.square(difference_y)) + np.sum(np.square(difference_z))
    return total_error

def reset_simulation(sim, model):
    sim.data.qpos[:] = model.key_qpos  # 重置位置
    sim.data.qvel[:] = model.key_qvel  # 重置速度
    sim.forward()  # 重新计算所有的状态

def optimize_length(initial_length, theMouse, theController, run_steps_num, time_step, max_iterations=50, learning_rate=0.001, tolerance=1e-4):
    length = initial_length
    best_length = length
    best_error = float('inf')
    error_history = []
    model, sim = load_model()

    for iteration in range(max_iterations):

        # 运行仿真并计算误差
        error = run_simulation(sim, length, theController, theMouse, run_steps_num, time_step)
        error_history.append(error)

        print(f"Iteration {iteration + 1}: Length = {length}, Error = {error}")

        # 如果误差更小，则更新最佳长度
        if error < best_error:
            best_error = error
            best_length = length

        # 重置仿真状态以开始新的迭代
        reset_simulation(sim, model)

        # 使用误差更新长度
        length -= learning_rate * error

        if error < tolerance:
            print(f"Converged after {iteration + 1} iterations.")
            break

    return best_length, best_error, error_history

def main():
    run_steps_num = 2500  # 根据需要调整
    time_step = 0.002
    initial_length = 0.0364  # 初始长度
    theMouse = SimModel("../models/dynamic_4l.xml", render_flag=True)
    theController = MouseController(fre=0.8, time_step=time_step, spine_angle=15)

    best_length, best_error, error_history = optimize_length(initial_length, theMouse, theController, run_steps_num, time_step)

    print(f"Best Length: {best_length}, Best Error: {best_error}")

    # 绘制误差历史
    plt.figure(figsize=(8, 6))
    plt.plot(error_history, label='Error History')
    plt.xlabel('Iteration')
    plt.ylabel('Error')
    plt.title('Error Convergence')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
