import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R

def exomujoco (obj, emptyXMLfile, mesh_dir, whichloaded):
    tree = ET.parse(emptyXMLfile)
    root = tree.getroot()

    # IMU and positional data
    pelvis_pos =[-0.16445, -0.014625,0.19109]
    pelvis_quat=[0.5,-0.5,-0.5,0.5]
    thorax_pos =[-0.14,0,0.5]
    thorax_quat=[0.70711,0,0.70711,0]
    left_tibia_pos =[-0.14067,-0.074092,-0.39348]
    left_tibia_quat =[2.3108e-07,-2.3108e-07,0.70711,-0.70711]
    right_tibia_pos=[-0.11573,0.074342,-0.39348]
    right_tibia_quat=[0.70711,-0.70711,0,0]
    left_foot_pos=[-0.036966,0.015,-0.13705]
    left_foot_quat=[-1.6377e-07,-0.94552,-5.639e-08,0.32557]
    right_foot_pos=[0.036966,0.015,-0.13705]
    right_foot_quat=[-1.6377e-07,-0.94552,-5.639e-08,0.32557]

    size_node = root.find('size')
    asset_node = root.find('asset')
    world_node = root.find('worldbody')
    compiler_node = root.find('compiler')

    compiler_node.set('radian',angle,'true',autolimits)

    # Modify size node attributes
    if size_node is not None:
        size_node.set('njmax', '500')
        size_node.set('nconmax', '100')

    default_node = ET.SubElement(root, 'default')
    joint_node = ET.SubElement(default_node, 'joint')
    joint_node.set('limited', 'false')

    eq_param_node = ET.SubElement(default_node, 'equality')
    eq_param_node.set('solref', '0.02 1')
    eq_param_node.set('solimp', '0.9 0.95 0.001 0.4 2')

    # Add solver and integration options
    mujoco_option = ET.SubElement(root, 'option')
    mujoco_option.set('timestep', '0.001')
    mujoco_option.set('iterations', '5')
    mujoco_option.set('ls_iterations', '20')
    mujoco_option.set('solver', 'Newton')
    mujoco_option.set('gravity', '0 0 0')
    mujoco_option.set('integrator', 'Euler')

    # Add visual settings
    mujoco_visual = ET.SubElement(root, 'visual')
    headlight = ET.SubElement(mujoco_visual, 'headlight')
    headlight.set('ambient', '0.5 0.5 0.5')
    headlight.set('diffuse', '0.5 0.5 0.5')
    headlight.set('specular', '0.5 0.5 0.5')
    map_element = ET.SubElement(mujoco_visual, 'map')
    map_element.set('force', '0.005')
    rgba = ET.SubElement(mujoco_visual, 'rgba')
    rgba.set('contactforce', '0.7 0.9 0.9 0.5')

    # Add plane texture and material to the asset node
    plane_texture = ET.SubElement(asset_node, 'texture')
    plane_texture.set('name', 'plane')
    plane_texture.set('type', '2d')
    plane_texture.set('builtin', 'checker')
    plane_texture.set('rgb1', '0.9 0.9 0.9')
    plane_texture.set('rgb2', '0.7 0.7 0.7')
    plane_texture.set('width', '512')
    plane_texture.set('height', '512')

    plane_material = ET.SubElement(asset_node, 'material')
    plane_material.set('name', 'plane')
    plane_material.set('reflectance', '0.5')
    plane_material.set('shininess', '1')
    plane_material.set('specular', '1')
    plane_material.set('texture', 'plane')
    plane_material.set('texrepeat', '60 60')

    plane_hfield = ET.SubElement(asset_node, 'hfield')
    plane_hfield.set('name','terrain')
    plane_hfield.set('size','7 7 1 0.001')
    plane_hfield.set('nrow','100')
    plane_hfield.set('ncol','100')

    # Modify worldbody
    world_new = ET.Element('worldbody')
    geom_plane = ET.SubElement(world_new, 'geom')
    geom_plane.set('name', 'terrain_mesh')
    geom_plane.set('type', 'plane')
    geom_plane.set('material', 'plane')
    geom_plane.set('size', '10 5 0.1')
    geom_plane.set('rgba', '.9 .9 .9 1')
    geom_plane.set('condim', '3')
    geom_plane.set('conaffinity', '3')
    geom_plane.set('friction', '1 .1 .1')

    torso_node = ET.SubElement(world_new, 'body')
    torso_node.set('name', 'torso')
    torso_node.set('pos', '0 0 1')

    # Add inertial properties to the torso
    if whichloaded.lower() == 'loaded':
        temp = obj.loadedExo.Bodies[6]
    elif whichloaded.lower() == 'empty':
        temp = obj.emptyExo.Bodies[6]
    else:
        raise ValueError("3rd argument must either be 'empty' or 'loaded'.")

    inertial = ET.SubElement(torso_node, 'inertial')
    inertial.set('pos', ' '.join(map(str, temp.CenterOfMass)))
    inertial.set('mass', str(temp.Mass))

    inertialExo = dynamics.Inertia(-temp.Mass, [0, 0, 0], temp.Inertia)
    IGExo = inertialExo.inertiaAt(temp.CenterOfMass, np.eye(3))
    inertial.set('diaginertia', ' '.join(map(str, IGExo.diagonal())))

    torso_geom = ET.SubElement(torso_node, 'geom')
    torso_geom.set('type', 'mesh')
    torso_geom.set('contype', '0')
    torso_geom.set('conaffinity', '0')
    torso_geom.set('group', '1')
    torso_geom.set('density', '0')
    torso_geom.set('mesh', 'PelvisLink')
    torso_geom.set('name','pelvismesh')

    # torso_free_joint = ET.SubElement(torso_node, 'freejoint')
    # torso_free_joint.set('name', 'root')

    # Add IMUs
    pelvis_imu = ET.SubElement(torso_node, 'site')
    pelvis_imu.set('name', 'pelvis_imu')
    pelvis_imu.set('group', '3')
    pelvis_imu.set('pos', ' '.join(map(str, pelvis_pos)))
    # pelvis_quat = R.from_euler('xyz', pelvis_rpy).as_quat()
    pelvis_imu.set('quat', ' '.join(map(str, pelvis_quat)))
    pelvis_imu.set('size', '.01')

    thorax_imu = ET.SubElement(torso_node, 'site')
    thorax_imu.set('name', 'thorax_imu')
    thorax_imu.set('group', '3')
    thorax_imu.set('pos', ' '.join(map(str, thorax_pos)))
    # thorax_quat = R.from_euler('xyz', thorax_rpy).as_quat()
    thorax_imu.set('quat', ' '.join(map(str, thorax_quat)))
    thorax_imu.set('size', '.01')

    torso_node.append(torso_geom)
    torso_node.append(torso_free_joint)
    torso_node.append(pelvis_imu)
    torso_node.append(thorax_imu)

    root.append(world_new)

    # Add contact pairs for feet
    contact = ET.SubElement(root, 'contact')
    for whichgeom1 in ['left_sole', 'right_sole', 'left_toe', 'right_toe', 'left_heel', 'right_heel']:
        pair = ET.SubElement(contact, 'pair')
        pair.set('geom1', whichgeom1)
        pair.set('geom2', 'plane')
        pair.set('margin', '0.0015')
        pair.set('condim', '4')
        pair.set('friction', '1 1 0.005 0.0001 0.0001')
        pair.set('contype', '1')
        pair.set('conaffinity', '0')
        pair.set('quat', '0.707105 0 -0.707108 0')
        pair.set('type', 'box')
    
    # Add IMU sites to ankles
    left_tibia_imu = ET.SubElement(root, 'site')
    left_tibia_imu.set('name', 'left_tibia_imu')
    left_tibia_imu.set('group', '3')
    left_tibia_imu.set('pos', ' '.join(map(str, left_tibia_pos)))
    # left_tibia_quat = R.from_euler('xyz', left_tibia_rpy).as_quat()
    left_tibia_imu.set('quat', ' '.join(map(str, left_tibia_quat)))
    left_tibia_imu.set('size', '.01')

    right_tibia_imu = ET.SubElement(root, 'site')
    right_tibia_imu.set('name', 'right_tibia_imu')
    right_tibia_imu.set('group', '3')
    right_tibia_imu.set('pos', ' '.join(map(str, right_tibia_pos)))
    # right_tibia_quat = R.from_euler('xyz', right_tibia_rpy).as_quat()
    right_tibia_imu.set('quat', ' '.join(map(str, right_tibia_quat)))
    right_tibia_imu.set('size', '.01')

    left_saggittal_knee_node = root.findall(".//body[@name='LeftSagittalKneeLink']")[0]
    left_saggittal_knee_node.append(left_tibia_imu)

    right_saggittal_knee_node = root.findall(".//body[@name='RightSagittalKneeLink']")[0]
    right_saggittal_knee_node.append(right_tibia_imu)

    # Add left foot add-ons
    left_henke_node = root.findall(".//body[@name='LeftHenkeAnkleLink']")[0]
    left_collision_node = left_henke_node.findall("./geom[@type='box']")[0]
    left_collision_node.set('size', '0.0049535 0.059645 0.1375')
    left_collision_node.set('rgba', '1 0 0 1')
    left_collision_node.set('condim', '4')
    left_collision_node.set('name', 'left_sole')
    left_collision_node.set('friction', '1')
    left_collision_node.set('pos', '0.06 -3.3e-05 -0.15975')

    left_collision_toe = ET.SubElement(left_henke_node, 'geom')
    left_collision_toe.set('condim', '4')
    left_collision_toe.set('friction', '1')
    left_collision_toe.set('name', 'left_toe')
    left_collision_toe.set('pos', '0.2195 0 -0.1537')
    left_collision_toe.set('quat', '0.6018 0 -0.7986 0')
    left_collision_toe.set('rgba', '1 0 0 1')
    left_collision_toe.set('size', '0.0049535 0.059645 0.022')
    left_collision_toe.set('type', 'box')

    left_collision_heel = ET.SubElement(left_henke_node, 'geom')
    left_collision_heel.set('condim', '4')
    left_collision_heel.set('friction', '1')
    left_collision_heel.set('name', 'left_heel')
    left_collision_heel.set('pos', '-0.0940 0 -0.1566')
    left_collision_heel.set('quat', '0.7716 0 -0.6361 0')
    left_collision_heel.set('rgba', '1 0 0 1')
    left_collision_heel.set('size', '0.0049535 0.059645 0.0165')
    left_collision_heel.set('type', 'box')

    foot_imu = ET.SubElement(left_henke_node, 'site')
    foot_imu.set('name', 'left_foot_imu')
    foot_imu.set('pos', ' '.join(map(str, left_foot_pos)))
    # foot_quat = R.from_euler('xyz', foot_rpy).as_quat()
    foot_imu.set('quat', ' '.join(map(str, left_foot_quat)))
    foot_imu.set('size', '.01')

    for i in range(1, 5):
        opto = ET.SubElement(left_henke_node, 'site')
        opto.set('name', f'opto{i}')
        opto.set('pos', ' '.join(map(str, [ExoConstants.dimensions.optoforce.pos[:, i-1], -ExoConstants.dimensions.toeHeight])))
        opto.set('quat', '1 0 0 1')
        opto.set('size', '0.05')
        opto.set('group', '2')
    
    # Add right foot add-ons
    right_henke_node = root.findall(".//body[@name='RightHenkeAnkleLink']")[0]
    right_collision_node = right_henke_node.findall("./geom[@type='box']")[0]
    right_collision_node.set('size', '0.0049535 0.059645 0.1375')
    right_collision_node.set('rgba', '1 0 0 1')
    right_collision_node.set('condim', '4')
    right_collision_node.set('name', 'right_sole')
    right_collision_node.set('friction', '1')
    right_collision_node.set('pos', '0.06 -3.3e-05 -0.15975')

    right_collision_toe = ET.SubElement(right_henke_node, 'geom')
    right_collision_toe.set('condim', '4')
    right_collision_toe.set('friction', '1')
    right_collision_toe.set('name', 'right_toe')
    right_collision_toe.set('pos', '0.2195 0 -0.1537')
    right_collision_toe.set('quat', '0.6018 0 -0.7986 0')
    right_collision_toe.set('rgba', '1 0 0 1')
    right_collision_toe.set('size', '0.0049535 0.059645 0.022')
    right_collision_toe.set('type', 'box')

    right_collision_heel = ET.SubElement(right_henke_node, 'geom')
    right_collision_heel.set('condim', '4')
    right_collision_heel.set('friction', '1')
    right_collision_heel.set('name', 'right_heel')
    right_collision_heel.set('pos', '-0.0940 0 -0.1566')
    right_collision_heel.set('quat', '0.7716 0 -0.6361 0')
    right_collision_heel.set('rgba', '1 0 0 1')
    right_collision_heel.set('size', '0.0049535 0.059645 0.0165')
    right_collision_heel.set('type', 'box')

    right_foot_imu = ET.SubElement(right_henke_node, 'site')
    right_foot_imu.set('name', 'right_foot_imu')
    right_foot_imu.set('pos', ' '.join(map(str, right_foot_pos)))
    right_foot_imu.set('quat', ' '.join(map(str, right_foot_quat)))
    right_foot_imu.set('size', '.01')

    for i in range(1, 5):
        opto = ET.SubElement(right_henke_node, 'site')
        opto.set('name', f'opto{i+4}')
        opto.set('pos', ' '.join(map(str, [ExoConstants.dimensions.optoforce.pos[:, i-1], -ExoConstants.dimensions.toeHeight])))
        opto.set('quat', '1 0 0 1')
        opto.set('size', '0.05')
        opto.set('group', '2')
    
    # Add actuators
    actuator_node = ET.SubElement(root, 'actuator')
    labels = ExoConstants.labels.joints
    limits = ExoConstants.limits.torque
    for i in range(len(labels)):
        motor = ET.SubElement(actuator_node, 'motor')
        motor_name = f"{labels[i]}Link_actuator"
        joint_name = f"{labels[i]}Joint"
        motor.set('name', motor_name)
        motor.set('joint', joint_name)
        motor.set('ctrlrange', f"{-limits[i]} {limits[i]}")
        motor.set('ctrllimited', 'true')
    
    
    #Add equality


    # Add sensors
    sensor_node = ET.SubElement(root, 'sensor')

    # Add 8 optoforce sensors on feet
    for i in range(1, 9):
        opto = ET.SubElement(sensor_node, 'touch')
        opto.set('name', f'opto{i}')
        opto.set('site', f'opto{i}')
    
    #add sensor for muscle model
    muscle_sites =['r_foot','r_toes','l_foot','l_toes']
    for site in muscle_sites:
        musclesensor= ET.SubElement(sensor_node, 'touch')
        musclesensor.set('name',f'{site}')
        musclesensor.set('site',f'{site}_touch')

    # Add IMU sensors
    imu_sites = ['pelvis', 'thorax', 'left_tibia', 'right_tibia', 'left_foot', 'right_foot']
    for site in imu_sites:
        imu_ori = ET.SubElement(sensor_node, 'framequat')
        imu_ori.set('name', f"{site}-orientation")
        imu_ori.set('objtype', 'site')
        imu_ori.set('objname', f"{site}_imu")
        
        imu_gyro = ET.SubElement(sensor_node, 'gyro')
        imu_gyro.set('name', f"{site}-angular-velocity")
        imu_gyro.set('site', f"{site}_imu")
        imu_gyro.set('noise', '5e-4')
        
        imu_accel = ET.SubElement(sensor_node, 'accelerometer')
        imu_accel.set('name', f"{site}-linear-acceleration")
        imu_accel.set('site', f"{site}_imu")
        imu_accel.set('noise', '1e-2')

    # Write to file
    tree.write(exowtendonXMLfile)



















