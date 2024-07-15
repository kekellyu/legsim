import mujoco
import numpy as np

# Define the joint names
joint_names = [
    "LeftFrontalHipJoint", "LeftTransverseHipJoint", "LeftSagittalHipJoint", "LeftSagittalKneeJoint", "LeftSagittalAnkleJoint",
    "LeftHenkeAnkleJoint", "RightFrontalHipJoint", "RightTransverseHipJoint", "RightSagittalHipJoint", "RightSagittalKneeJoint",
    "RightSagittalAnkleJoint", "RightHenkeAnkleJoint", "hip_flexion_r", "hip_adduction_r", "hip_rotation_r",
    "knee_angle_r_translation2", "knee_angle_r_translation1", "knee_angle_r", "knee_angle_r_rotation2", "knee_angle_r_rotation3",
    "ankle_angle_r", "subtalar_angle_r", "mtp_angle_r", "knee_angle_r_beta_translation2", "knee_angle_r_beta_translation1",
    "knee_angle_r_beta_rotation1", "hip_flexion_l", "hip_adduction_l", "hip_rotation_l",
    "knee_angle_l_translation2", "knee_angle_l_translation1", "knee_angle_l", "knee_angle_l_rotation2", "knee_angle_l_rotation3",
    "ankle_angle_l", "subtalar_angle_l", "mtp_angle_l", "knee_angle_l_beta_translation2", "knee_angle_l_beta_translation1",
    "knee_angle_l_beta_rotation1"
]

model = mujoco.MjModel.from_xml_path("generate_mujoco_model.xml")
data = mujoco.MjData(model)

num_steps = 1000
qpos_trajectory_data = []
qvel_trajectory_data = []

def update_and_store(model, data, qpos_trajectory_data, qvel_trajectory_data):
    mujoco.mj_step(model, data)
    qpos = data.qpos.copy()
    qvel = data.qvel.copy()

    qpos_trajectory_data.append(qpos)
    qvel_trajectory_data.append(qvel)


for i in range(num_steps):
    update_and_store(model, data, qpos_trajectory_data, qvel_trajectory_data)

qpos_trajectory_data = np.array(qpos_trajectory_data)
qvel_trajectory_data = np.array(qvel_trajectory_data)

qpos_header = ",".join(joint_names)
qvel_header = ",".join(joint_names)

np.savetxt("qpos.csv", qpos_trajectory_data, delimiter=",", header=qpos_header, comments='')
np.savetxt("qvel.csv", qvel_trajectory_data, delimiter=",", header=qvel_header, comments='')

print("data saved to csv.")
