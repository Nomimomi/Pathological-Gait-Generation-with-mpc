import mujoco_py

import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from LegModel.legs import LegModel

model = mujoco_py.load_model_from_path("../../models/dynamic_4l.xml")
sim = mujoco_py.MjSim(model)

def print_mass_list(model,sim):
    n_bodies = model.nbody
    for body_id in range(n_bodies-1):
        mass = sim.model.body_mass[body_id]
        if mass > 0:
            body_name = model.body_id2name(body_id)
            print(f"Body ID: {body_id}, Name: {body_name}, Mass: {mass}")


print_mass_list(model,sim)