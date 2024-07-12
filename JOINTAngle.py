import math

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

# Assign data to joints and convert to degrees
joint_angles = {}
for i, value in enumerate(qpos_data):
    joint_angles[joint_names[i]] = rad_to_deg(value)

for joint, angle in joint_angles.items():
    print(f"{joint}: {angle:.2f} degrees")

RightHenkeAnkleJoint_angle = joint_angles["RightHenkeAnkleJoint"]
ankle_angle_r_angle = joint_angles["ankle_angle_r"]
angle_difference = abs(RightHenkeAnkleJoint_angle - ankle_angle_r_angle)

print(f"\nDifference between ankle: {angle_difference:.2f} degrees")