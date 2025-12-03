import mujoco
from mujoco import viewer
import numpy as np
import os
from lxml import etree
import time

# ---------- Matplotlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------- MuJoCo 模型路径 ----------

f1 = "/Users/zhangx1ang/VS_Code/SRS_Task3,4_tendon/tendon-connected-2R__.xml"
f2 = "/Users/zhangx1ang/VS_Code/SRS_Task3,4_tendon/tendon-connected-2R_modified__.xml"

# parameter
R1, R2, a, b, c = 0.017, 0.028, 0.088, 0.089, 0.032

import xml.etree.ElementTree as ET

def swap_par(tree, element_type, element_name, attr, new_value):

    element = tree.find(f'.//{element_type}[@name="{element_name}"]')
    if element is not None:
        element.set(attr, str(new_value))
    else:
        print(f"Element not found: {element_type} name={element_name}")


# ---------- 修改并保存 XML ----------
tree = ET.parse(f1)
root = tree.getroot()

# link1
swap_par(root, 'body', 'link1', 'pos', f"{a} 0 0")
swap_par(root, 'geom', 'link1_geom', 'fromto', f"0 0 0 {c} 0 0")

# link2
swap_par(root, 'body', 'link2', 'pos', f"{c} 0 0")
swap_par(root, 'geom', 'link2_geom', 'fromto', f"0 0 0 {b} 0 0")

# tendon sites
swap_par(root, 'site', 'sR1_top', 'pos', f"{c}  {R1 * 3} 0")
swap_par(root, 'site', 'sR1_bottom', 'pos', f"{c} -{R1 * 3} 0")
swap_par(root, 'site', 'sR2_top', 'pos', f"0.0  {R2 * 3} 0")
swap_par(root, 'site', 'sR2_bottom', 'pos', f"0.0 -{R2 * 3} 0")

# anchor
x_right = a + c + b
swap_par(root, 'site', 'anchor_right_top', 'pos', f"{x_right}  0.05 0")
swap_par(root, 'site', 'anchor_right_bottom', 'pos', f"{x_right} -0.05 0")

# save XML
tree.write(f2, encoding='utf-8', xml_declaration=True)
print("xml updated and saved", f2)

# 检查生成情况
print("文件是否存在？", os.path.exists(f2))
if not os.path.exists(f2):
    raise FileNotFoundError(f"未找到 {f2}，请检查路径或写入权限！")


# ---------- Loading model ----------
model = mujoco.MjModel.from_xml_path(f2)
data = mujoco.MjData(model)


# ---------- Control logic----------
def set_torque(mj_data, KP, KV, theta):
    mj_data.ctrl[0] = KP * (-mj_data.qpos[0] + theta) + KV * (0 - mj_data.qvel[0])



SIMEND = 20
TIMESTEP = 0.01
STEP_NUM = int(SIMEND / TIMESTEP)
timeseries = np.linspace(0, SIMEND, STEP_NUM)

T = 2  # [s]
FREQ = 1 / T
AMP = np.deg2rad(-90)
BIAS = np.deg2rad(-90)

theta_des = AMP * np.sin(FREQ * timeseries) + BIAS

#  record datas
EE_position_x, EE_position_z = [], []

#   MuJoCo Viewer
with viewer.launch_passive(model, data) as v:
    t0 = time.time()
    while v.is_running() and data.time - t0 < 5.0:  # 模拟 5 秒
        t = data.time - t0

        # Apply periodic control to the first joint
        data.ctrl[0] = 0.3 * np.sin(2 * np.pi * 0.5 * t)

        mujoco.mj_step(model, data)
        v.sync()

        # Get the location of the end effector site
        x_ee = data.site_xpos[model.site('sR2_top').id][0]
        z_ee = data.site_xpos[model.site('sR2_top').id][2]

        EE_position_x.append(x_ee)
        EE_position_z.append(z_ee)


# Drawing trajectory
print(f"Matplotlib backend: {plt.get_backend()}")

plt.figure(figsize=(6, 6))
plt.plot(EE_position_x, EE_position_z, color='b')
plt.xlabel("x [m]")
plt.ylabel("z [m]")
plt.title("End-Effector Trajectory")
plt.grid(True)
plt.tight_layout()
plt.savefig("ee_trajectory.png", dpi=200)
print("✅ 轨迹图已保存为 ee_trajectory.png")
