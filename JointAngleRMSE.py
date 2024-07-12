import math
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

# Convert radians to degrees
def rad_to_deg(radians):
    return radians * 180 / math.pi

# Calculate RMSE
def calculate_gait_rmse(joint1_angles, joint2_angles):
    differences = abs(np.array(joint1_angles)) - abs(np.array(joint2_angles))
    mse = np.mean(differences)
    rmse = np.sqrt(mse)
    return rmse

# Calculate the sum of RMSE for multiple joint pairs
def calculate_sum_rmse(joint_pairs, joint_angles):
    sum_rmse = 0
    for joint1, joint2 in joint_pairs:
        rmse = calculate_gait_rmse(joint_angles[joint1], joint_angles[joint2])
        sum_rmse += rmse
    return sum_rmse

# Read the file and process QPOS data
qpos_data = []
qpos_section = False
with open('MJDATA.TXT', 'r') as file:
    for line in file:
        if "QPOS" in line:
            qpos_section = True
            continue
        if qpos_section:
            if line.strip() == "":
                break
            qpos_data.append(float(line.strip()))

joint_angles = {}
for i, value in enumerate(qpos_data):
    joint_angles[joint_names[i]] = rad_to_deg(value)

joint_pairs = [
    ("RightHenkeAnkleJoint", "ankle_angle_r"),
    ("LeftHenkeAnkleJoint", "ankle_angle_l"),
]

for joint, angle in joint_angles.items():
    print(f"{joint}: {angle:.2f} degrees")

sum_rmse = calculate_sum_rmse(joint_pairs, joint_angles)
print(f"Sum of RMSE for the defined joint pairs: {sum_rmse:.2f} degrees")