import xml.etree.ElementTree as ET
import math

def load_xml_file_to_tree(xml_file):
    try:
        tree = ET.parse(xml_file)
        root = tree.getroot()
        return tree, root
    except ET.ParseError as e:
        print(f"Error parsing XML file: {e}")
        return None, None
    except FileNotFoundError:
        print("The specified XML file was not found.")
        return None, None

def get_elements(root, tag):
    return [element for element in root.iter(tag)]

def mod_mujoco_xml(obj, mujocoXMLfile, mesh_dir, whichloaded,emptyXMLfile):
    tree, root = load_xml_file_to_tree(mujocoXMLfile)
    if root is None:
        return

    # Load from the empty.xml file
    empty_tree, empty_root = load_xml_file_to_tree(emptyXMLfile)
    if empty_root is None:
        return

    actuators = get_elements(empty_root, 'general')
    tendons = get_elements(empty_root, 'spatial')
    sites = get_elements(empty_root, 'site')
    equality_elements = get_elements(empty_root, 'equality')

    # IMU positions and orientations
    pelvis_rpy = [-1.570796, 0.0, 1.570796]
    pelvis_pos = [-0.164445, -0.014625, 0.191095]
    thorax_rpy = [0.0, 1.570796, 0.0]
    thorax_pos = [-0.14, 0.0, 0.5]
    left_tibia_rpy = [-1.570796, -0.0, -3.141592]
    left_tibia_pos = [-0.140666, -0.074092, -0.393475]
    right_tibia_rpy = [-1.570796, -0.0, 0.0]
    right_tibia_pos = [-0.11573, 0.074342, -0.393475]
    foot_rpy = [-3.141593, 0.663225, 0.0]
    foot_pos = [-0.036966, 0.015, -0.13705]

    # Add meshdir path
    compiler_node = root.find('compiler')
    compiler_node.set('meshdir', mesh_dir)

    # Modify size
    size_node = root.find('size')
    if size_node is not None:
        size_node.set('njmax', '500')
        size_node.set('nconmax', '100')

    # Modify default settings
    default_node = ET.SubElement(root, 'default')
    joint_node = ET.SubElement(default_node, 'joint')
    joint_node.set('limited', 'false')
    eq_param_node = ET.SubElement(default_node, 'equality')
    eq_param_node.set('solref', '0.02 1')
    eq_param_node.set('solimp', '0.9 0.95 0.001 0.4 2')
    default_class_node = ET

    # Create the myoleg default node
    default_myolegs = ET.SubElement(root, 'default')
    default_myolegs.set('class', 'myolegs')

    # Add joint element
    joint_node = ET.SubElement(default_myolegs, 'joint')
    joint_node.set('armature', '0.01')
    joint_node.set('damping', '0.5')
    joint_node.set('limited', 'true')

    # Add geom element
    geom_node = ET.SubElement(default_myolegs, 'geom')
    geom_node.set('margin', '0.001')
    geom_node.set('material', 'mat_myolegs')
    geom_node.set('rgba', '0.8 0.85 0.8 1')
    geom_node.set('conaffinity', '0')
    geom_node.set('contype', '0')

    # Add site element
    site_node = ET.SubElement(default_myolegs, 'site')
    site_node.set('size', '0.001 0.005 0.005')
    site_node.set('group', '3')

    # Add nested default node with class "muscle"
    default_muscle = ET.SubElement(default_myolegs, 'default')
    default_muscle.set('class', 'muscle')
    general_node = ET.SubElement(default_muscle, 'general')
    general_node.set('biasprm', '0.75 1.05 -1 400 0.5 1.6 1.5 1.3 1.2 0')
    general_node.set('biastype', 'muscle')
    general_node.set('ctrllimited', 'true')
    general_node.set('ctrlrange', '0 1')
    general_node.set('dynprm', '0.01 0.04 0 0 0 0 0 0 0 0')
    general_node.set('dyntype', 'muscle')
    general_node.set('gainprm', '0.75 1.05 -1 400 0.5 1.6 1.5 1.3 1.2 0')
    general_node.set('gaintype', 'muscle')

    # Add nested default node with class "wrap"
    default_wrap = ET.SubElement(default_myolegs, 'default')
    default_wrap.set('class', 'wrap')
    wrap_geom_node = ET.SubElement(default_wrap, 'geom')
    wrap_geom_node.set('rgba', '0.5 0.5 0.9 0.5')
    wrap_geom_node.set('group', '3')
    wrap_geom_node.set('contype', '0')
    wrap_geom_node.set('conaffinity', '0')
    wrap_geom_node.set('type', 'cylinder')

    # Add nested default node with class "coll"
    default_coll = ET.SubElement(default_myolegs, 'default')
    default_coll.set('class', 'coll')
    coll_geom_node = ET.SubElement(default_coll, 'geom')
    coll_geom_node.set('type', 'capsule')
    coll_geom_node.set('group', '1')
    coll_geom_node.set('contype', '1')
    coll_geom_node.set('conaffinity', '0')
    coll_geom_node.set('condim', '3')
    coll_geom_node.set('rgba', '0.8 0.7 0.5 1')
    coll_geom_node.set('margin', '0.001')
    coll_geom_node.set('material', 'MatSkin')

    # Add nested default node with class "myo_leg_touch"
    default_myo_leg_touch = ET.SubElement(default_myolegs, 'default')
    default_myo_leg_touch.set('class', 'myo_leg_touch')
    touch_site_node = ET.SubElement(default_myo_leg_touch, 'site')
    touch_site_node.set('type', 'box')
    touch_site_node.set('group', '3')
    touch_site_node.set('rgba', '0.8 0.2 0.2 0.4')

    # Add nested default node with class "myo_leg_marker"
    default_myo_leg_marker = ET.SubElement(default_myolegs, 'default')
    default_myo_leg_marker.set('class', 'myo_leg_marker')
    marker_site_node = ET.SubElement(default_myo_leg_marker, 'site')
    marker_site_node.set('size', '0.02')
    marker_site_node.set('group', '4')
    marker_site_node.set('rgba', '0.8 0.8 0.2 1')

    # Add solver/integration setup options
    mujoco_option = ET.Element('option')
    root.insert(list(root).index(root.find('asset')), mujoco_option)
    mujoco_option.set('timestep', '0.001')
    mujoco_option.set('iterations', '50')
    mujoco_option.set('solver', 'Newton')
    mujoco_option.set('gravity', '0 0 -9.81')
    mujoco_option.set('integrator', 'RK4')

    # Add visual settings
    mujoco_visual = ET.Element('visual')
    root.insert(list(root).index(root.find('asset')), mujoco_visual)
    headlight = ET.SubElement(mujoco_visual, 'headlight')
    headlight.set('ambient', '0.5 0.5 0.5')
    map_node = ET.SubElement(mujoco_visual, 'map')
    map_node.set('force', '0.005')
    rgba_node = ET.SubElement(mujoco_visual, 'rgba')
    rgba_node.set('contactforce', '0.7 0.9 0.9 0.5')

    # Add plane texture/material to asset
    asset_node = root.find('asset')
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
    plane_material.set('reflectance', '0.0')
    plane_material.set('texture', 'plane')
    plane_material.set('texrepeat', '1 1')
    plane_material.set('texuniform', 'true')

    # Modify worldbody
    world_node = root.find('worldbody')
    body_left = world_node.find('body')
    body_clone_left = ET.fromstring(ET.tostring(body_left))
    body_right = world_node.findall('body')[6]
    body_clone_right = ET.fromstring(ET.tostring(body_right))

    # Remove worldbody and add a new one
    root.remove(world_node)
    world_new = ET.Element('worldbody')

    # Add plane geom
    geom_plane = ET.SubElement(world_new, 'geom')
    geom_plane.set('name', 'plane')
    geom_plane.set('type', 'plane')
    geom_plane.set('material', 'plane')
    geom_plane.set('size', '10 5 0.1')
    geom_plane.set('rgba', '0.9 0.9 0.9 1')
    geom_plane.set('condim', '3')
    geom_plane.set('conaffinity', '15')
    geom_plane.set('friction', '1')

    # Add torso
    torso_node = ET.SubElement(world_new, 'body')
    torso_node.set('name', 'torso')
    torso_node.set('pos', '0 0 1')

    if whichloaded.lower() == 'loaded':
        temp = obj.loadedExo.Bodies[6]
    elif whichloaded.lower() == 'empty':
        temp = obj.emptyExo.Bodies[6]
    else:
        raise ValueError("3rd argument must either be 'empty' or 'loaded'.")

    # Set torso inertial
    inertial = ET.SubElement(torso_node, 'inertial')
    inertial.set('pos', ' '.join(map(str, temp.CenterOfMass)))
    inertial.set('mass', str(temp.Mass))

    inertialExo = dynamics.Inertia(-temp.Mass, [0, 0, 0], temp.Inertia)
    IGExo = inertialExo.inertiaAt(temp.CenterOfMass, np.eye(3))
    inertial.set('diaginertia', ' '.join(map(str, np.diag(IGExo))))

    torso_geom = ET.SubElement(torso_node, 'geom')
    torso_geom.set('type', 'mesh')
    torso_geom.set('contype', '0')
    torso_geom.set('conaffinity', '0')
    torso_geom.set('group', '1')
    torso_geom.set('density', '0')
    torso_geom.set('mesh', 'PelvisLink')

    #Add Sites
    torso_site = ET.SubElement(torso_node, 'site')
    torso_site.set('pelvissite','0 0 0')
    torso_site.set('LeftHipBack','-0.08 0.08 -0.04')
    torso_site.set('LeftHipFront','0.09 0.08 -0.04')
    torso_site.set('RightHipBack','-0.08 -0.08 -0.04')
    torso_site.set('RightHipFront','0.09 -0.08 -0.04')

    # Add joints
    torso_free_joint = ET.SubElement(torso_node, 'freejoint')
    torso_free_joint.set('name', 'root')

    # Add IMUs
    pelvis_imu = ET.SubElement(torso_node, 'site')
    pelvis_imu.set('name', 'pelvis_imu')
    pelvis_imu.set('group', '3')
    pelvis_imu.set('pos', ' '.join(map(str, pelvis_pos)))
    pelvis_quat = angle2quat(*pelvis_rpy)
    pelvis_imu.set('quat', ' '.join(map(str, pelvis_quat)))
    pelvis_imu.set('size', '.01')

    thorax_imu = ET.SubElement(torso_node, 'site')
    thorax_imu.set('name', 'thorax_imu')
    thorax_imu.set('group', '3')
    thorax_imu.set('pos', ' '.join(map(str, thorax_pos)))
    thorax_quat = angle2quat(*thorax_rpy)
    thorax_imu.set('quat', ' '.join(map(str, thorax_quat)))
    thorax_imu.set('size', '.01')

    torso_node.append(torso_geom)
    torso_node.append(torso_free_joint)
    torso_node.append(pelvis_imu)
    torso_node.append(thorax_imu)
    torso_node.append(body_clone_left)
    torso_node.append(body_clone_right)

    root.append(world_new)

    # Add contact pair for feet
    contact = ET.SubElement(root, 'contact')
    for whichgeom1 in ['left_sole', 'right_sole', 'left_toe', 'right_toe', 'left_heel', 'right_heel']:
        pair = ET.SubElement(contact, 'pair')
        pair.set('geom1', whichgeom1)
        pair.set('geom2', 'plane')
        pair.set('margin', '0.0015')
        pair.set('condim', '4')
        pair.set('friction', '1 1 0.005 0.0001 0.0001')

    # Add IMU sites to ankles
    left_tibia_imu = ET.Element('site')
    left_tibia_imu.set('name', 'left_tibia_imu')
    left_tibia_imu.set('group', '3')
    left_tibia_imu.set('pos', ' '.join(map(str, left_tibia_pos)))
    left_tibia_quat = angle2quat(*left_tibia_rpy)
    left_tibia_imu.set('quat', ' '.join(map(str, left_tibia_quat)))
    left_tibia_imu.set('size', '.01')

    right_tibia_imu = ET.Element('site')
    right_tibia_imu.set('name', 'right_tibia_imu')
    right_tibia_imu.set('group', '3')
    right_tibia_imu.set('pos', ' '.join(map(str, right_tibia_pos)))
    right_tibia_quat = angle2quat(*right_tibia_rpy)
    right_tibia_imu.set('quat', ' '.join(map(str, right_tibia_quat)))
    right_tibia_imu.set('size', '.01')

    left_saggittal_knee_node = root.findall('.//body[@name="LeftSagittalKneeLink"]')[0]
    knee_child_node = left_saggittal_knee_node.find('body')
    left_saggittal_knee_node.remove(knee_child_node)
    left_saggittal_knee_node.append(left_tibia_imu)
    left_saggittal_knee_node.append(knee_child_node)

    right_saggittal_knee_node = root.findall('.//body[@name="RightSagittalKneeLink"]')[0]
    knee_child_node = right_saggittal_knee_node.find('body')
    right_saggittal_knee_node.remove(knee_child_node)
    right_saggittal_knee_node.append(right_tibia_imu)
    right_saggittal_knee_node.append(knee_child_node)

    # Left foot add-ons
    left_henke_node = root.findall('.//body[@name="LeftHenkeAnkleLink"]')[0]
    left_collision_node = left_henke_node.findall('geom')[7]
    left_collision_node.set('size', '0.0049535 0.059645 0.1375')
    left_collision_node.set('rgba', '1 0 0 1')
    left_collision_node.set('condim', '4')
    left_collision_node.set('name', 'left_sole')
    left_collision_node.set('friction', '1')

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
    foot_imu.set('pos', ' '.join(map(str, foot_pos)))
    foot_quat = angle2quat(*foot_rpy)
    foot_imu.set('quat', ' '.join(map(str, foot_quat)))
    foot_imu.set('size', '.01')

    # Add bodies for each optoforce 'site'
    for i in range(1, 5):
        opto = ET.SubElement(left_henke_node, 'site')
        opto.set('name', f'opto{i}')
        opto.set('pos', ' '.join(map(str, [ExoConstants.dimensions.optoforce.pos[:, i-1], -ExoConstants.dimensions.toeHeight])))
        opto.set('quat', '1 0 0 1')
        opto.set('size', '0.05')
        opto.set('group', '2')

    # Right foot add-ons
    right_henke_node = root.findall('.//body[@name="RightHenkeAnkleLink"]')[0]
    right_collision_node = right_henke_node.findall('geom')[7]
    right_collision_node.set('size', '0.0049535 0.059645 0.1375')
    right_collision_node.set('rgba', '1 0 0 1')
    right_collision_node.set('condim', '4')
    right_collision_node.set('name', 'right_sole')
    right_collision_node.set('friction', '1')

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

    right_foot_imu = ET.Element('site')
    right_foot_imu = ET.fromstring(ET.tostring(foot_imu))
    right_foot_imu.set('name', 'right_foot_imu')
    right_henke_node.append(right_foot_imu)

    for i in range(1, 5):
        opto = ET.SubElement(right_henke_node, 'site')
        opto.set('name', f'opto{i+4}')
        opto.set('pos', ' '.join(map(str, [ExoConstants.dimensions.optoforce.pos[:, i-1], -ExoConstants.dimensions.toeHeight])))
        opto.set('quat', '1 0 0 1')
        opto.set('size', '0.05')
        opto.set('group', '2')

    # Actuators for exo
    # actuator_node = ET.SubElement(root, 'actuator')
    # labels = ExoConstants.labels.joints
    # limits = ExoConstants.limits.torque
    # for i in range(len(labels)):
    #     motor = ET.SubElement(actuator_node, 'motor')
    #     motor_name = f"{labels[i]}Link_actuator"
    #     joint_name = f"{labels[i]}Joint"
    #     motor.set('name', motor_name)
    #     motor.set('joint', joint_name)
    #     motor.set('ctrlrange', f"{-limits[i]} {limits[i]}")
    #     motor.set('ctrllimited', 'true')
    
    #Actuators
    actuator_node = ET.SubElement(root, 'actuator')
    for actuator in actuators:
        actuator_node.append(actuator)
    
    #Sites
    for site in sites:
        world_new.append(site)

    # Tendons
    tendon_node = root.find('tendon')
    if tendon_node is None:
        tendon_node = ET.SubElement(root, 'tendon')
    for tendon in tendons:
        tendon_node.append(tendon)

    # Sensors
    sensor_node = ET.SubElement(root, 'sensor')

    # Add 8 optoforce sensors on feet
    for i in range(1, 9):
        opto = ET.SubElement(sensor_node, 'touch')
        opto.set('name', f'opto{i}')
        opto.set('site', f'opto{i}')

    #Add equality
    equality_node = root.find('equality')
    if equality_node is None:
        equality_node = ET.SubElement(root, 'equality')
    for eq in equality_elements:
        equality_node.append(eq)

    #add sensor for muscle model
    muscle_sites =['r_foot','r_toes','l_foot','l_toes']
    for site in muscle_sites:
        musclesensor= ET.SubElement(sensor_node, 'touch')
        musclesensor.set('name',f'{site}')
        musclesensor.set('site',f'{site}_touch')

    # Add 6 imu sensors
    imu_sites = ['pelvis', 'thorax', 'left_tibia', 'right_tibia', 'left_foot', 'right_foot']
    for site in imu_sites:
        imu_ori = ET.SubElement(sensor_node, 'framequat')
        imu_ori.set('name', f'{site}-orientation')
        imu_ori.set('objtype', 'site')
        imu_ori.set('objname', f'{site}_imu')

        imu_gyro = ET.SubElement(sensor_node, 'gyro')
        imu_gyro.set('name', f'{site}-angular-velocity')
        imu_gyro.set('site', f'{site}_imu')
        imu_gyro.set('noise', '5e-4')

        imu_accel = ET.SubElement(sensor_node, 'accelerometer')
        imu_accel.set('name', f'{site}-linear-acceleration')
        imu_accel.set('site', f'{site}_imu')
        imu_accel.set('noise', '1e-2')

    tree.write(mujocoXMLfile, encoding='utf-8', xml_declaration=True)