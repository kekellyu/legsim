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
    
    # Tendon
    tendon_node = ET.SubElement(root,'tendon')

    tendon_righthip_node = ET.SubElement(tendon_node,'spatial')
    tendon_righthip_node.set('name','righthipconnect')
    tendon_righthip_node.set('limited','true')
    tendon_righthip_node.set('range','0 0.5')
    tendon_righthip_node.set('rgba','0 0 0 0.5')
    tendon_righthip_node.set('width','.005')
    tendon_righthip_site1= ET.SubElement(tendon_righthip_node,'site')
    tendon_righthip_site1.set=('site','RightSagittalHipSite')
    tendon_righthip_site2= ET.SubElement(tendon_righthip_node,'site')
    tendon_righthip_site2.set=('site','hip_r_location')
    tendon_righthip_site3= ET.SubElement(tendon_righthip_node,'site')
    tendon_righthip_site3.set=('site','RightTransverseHipSite')

    tendon_rightknee_node = ET.SubElement(tendon_node,'spatial')
    tendon_rightknee_node.set('name','rightkneeconnect')
    tendon_rightknee_node.set('limited','true')
    tendon_rightknee_node.set('range','0 0.45')
    tendon_rightknee_node.set('rgba','0 0 0 0.5')
    tendon_rightknee_node.set('width','.005')
    tendon_rightknee_site1= ET.SubElement(tendon_rightknee_node,'site')
    tendon_rightknee_site1.set=('site','RightSagittalKneeFront')
    tendon_rightknee_site2= ET.SubElement(tendon_rightknee_node,'site')
    tendon_rightknee_site2.set=('site','RightKneeFront')
    tendon_rightknee_site3= ET.SubElement(tendon_rightknee_node,'site')
    tendon_rightknee_site3.set=('site','knee_r_location')
    tendon_rightknee_site4= ET.SubElement(tendon_rightknee_node,'site')
    tendon_rightknee_site4.set=('site','RightKneeBack')
    tendon_rightknee_site5= ET.SubElement(tendon_rightknee_node,'site')
    tendon_rightknee_site5.set=('site','RightSagittalKneeBack')

    # rightankleconnect
    tendon_rightankle_node = ET.SubElement(tendon_node, 'spatial')
    tendon_rightankle_node.set('name', 'rightankleconnect')
    tendon_rightankle_node.set('limited', 'true')
    tendon_rightankle_node.set('range', '0 0.1')
    tendon_rightankle_node.set('rgba', '0 0 0 0.5')
    tendon_rightankle_node.set('width', '.005')
    sites_rightankle = ['RightHenkeAnkleFront', 'RightAnkleFront', 'ankle_r_location', 'RightAnkleBack', 'RightHenkeAnkleBack']
    for site in sites_rightankle:
        site_node = ET.SubElement(tendon_rightankle_node, 'site')
        site_node.set('site', site)

    # lefthipconnect
    tendon_lefthip_node = ET.SubElement(tendon_node, 'spatial')
    tendon_lefthip_node.set('name', 'lefthipconnect')
    tendon_lefthip_node.set('limited', 'true')
    tendon_lefthip_node.set('range', '0 0.5')
    tendon_lefthip_node.set('rgba', '0 0 0 0.5')
    tendon_lefthip_node.set('width', '.005')
    sites_lefthip = ['LeftSagittalHipSite', 'hip_l_location', 'LeftTransverseHipSite']
    for site in sites_lefthip:
        site_node = ET.SubElement(tendon_lefthip_node, 'site')
        site_node.set('site', site)

    # leftkneeconnect
    tendon_leftknee_node = ET.SubElement(tendon_node, 'spatial')
    tendon_leftknee_node.set('name', 'leftkneeconnect')
    tendon_leftknee_node.set('limited', 'true')
    tendon_leftknee_node.set('range', '0 0.45')
    tendon_leftknee_node.set('rgba', '0 0 0 0.5')
    tendon_leftknee_node.set('width', '.005')
    sites_leftknee = ['LeftSagittalKneeFront', 'LeftKneeFront', 'knee_l_location', 'LeftKneeBack', 'LeftSagittalKneeBack']
    for site in sites_leftknee:
        site_node = ET.SubElement(tendon_leftknee_node, 'site')
        site_node.set('site', site)

    # leftankleconnect
    tendon_leftankle_node = ET.SubElement(tendon_node, 'spatial')
    tendon_leftankle_node.set('name', 'leftankleconnect')
    tendon_leftankle_node.set('limited', 'true')
    tendon_leftankle_node.set('range', '0 0.1')
    tendon_leftankle_node.set('rgba', '0 0 0 0.5')
    tendon_leftankle_node.set('width', '.005')
    sites_leftankle = ['LeftHenkeAnkleFront', 'LeftAnkleFront', 'ankle_l_location', 'LeftAnkleBack', 'LeftHenkeAnkleBack']
    for site in sites_leftankle:
        site_node = ET.SubElement(tendon_leftankle_node, 'site')
        site_node.set('site', site)
    

    # addbrev_r_tendon
    tendon_addbrev_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addbrev_r_node.set('name', 'addbrev_r_tendon')
    tendon_addbrev_r_node.set('springlength', '0.0354503')
    sites_addbrev_r = ['addbrev_r-P1', 'addbrev_r-P2']
    for site in sites_addbrev_r:
        site_node = ET.SubElement(tendon_addbrev_r_node, 'site')
        site_node.set('site', site)

    # addlong_r_tendon
    tendon_addlong_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addlong_r_node.set('name', 'addlong_r_tendon')
    tendon_addlong_r_node.set('springlength', '0.131799')
    sites_addlong_r = ['addlong_r-P1', 'addlong_r-P2']
    for site in sites_addlong_r:
        site_node = ET.SubElement(tendon_addlong_r_node, 'site')
        site_node.set('site', site)

    # addmagDist_r_tendon
    tendon_addmagDist_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addmagDist_r_node.set('name', 'addmagDist_r_tendon')
    tendon_addmagDist_r_node.set('springlength', '0.0873807')
    sites_addmagDist_r = ['addmagDist_r-P1', 'addmagDist_r-P2']
    for site in sites_addmagDist_r:
        site_node = ET.SubElement(tendon_addmagDist_r_node, 'site')
        site_node.set('site', site)

    # addmagIsch_r_tendon
    tendon_addmagIsch_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addmagIsch_r_node.set('name', 'addmagIsch_r_tendon')
    tendon_addmagIsch_r_node.set('springlength', '0.216343')
    sites_addmagIsch_r = ['addmagIsch_r-P1', 'addmagIsch_r-P2']
    for site in sites_addmagIsch_r:
        site_node = ET.SubElement(tendon_addmagIsch_r_node, 'site')
        site_node.set('site', site)

    # addmagMid_r_tendon
    tendon_addmagMid_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addmagMid_r_node.set('name', 'addmagMid_r_tendon')
    tendon_addmagMid_r_node.set('springlength', '0.0466277')
    sites_addmagMid_r = ['addmagMid_r-P1', 'addmagMid_r-P2']
    for site in sites_addmagMid_r:
        site_node = ET.SubElement(tendon_addmagMid_r_node, 'site')
        site_node.set('site', site)

    # addmagProx_r_tendon
    tendon_addmagProx_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addmagProx_r_node.set('name', 'addmagProx_r_tendon')
    tendon_addmagProx_r_node.set('springlength', '0.0403241')
    sites_addmagProx_r = ['addmagProx_r-P1', 'addmagProx_r-P2']
    for site in sites_addmagProx_r:
        site_node = ET.SubElement(tendon_addmagProx_r_node, 'site')
        site_node.set('site', site)

    # bflh_r_tendon
    tendon_bflh_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_bflh_r_node.set('name', 'bflh_r_tendon')
    tendon_bflh_r_node.set('springlength', '0.325227')
    sites_bflh_r = ['bflh_r-P1', 'bflh_r-P2', 'bflh_r-P3']
    for site in sites_bflh_r:
        site_node = ET.SubElement(tendon_bflh_r_node, 'site')
        site_node.set('site', site)

    # bfsh_r_tendon
    tendon_bfsh_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_bfsh_r_node.set('name', 'bfsh_r_tendon')
    tendon_bfsh_r_node.set('springlength', '0.105817')
    sites_bfsh_r = ['bfsh_r-P1', 'bfsh_r-P2']
    for site in sites_bfsh_r:
        site_node = ET.SubElement(tendon_bfsh_r_node, 'site')
        site_node.set('site', site)

    # edl_r_tendon
    tendon_edl_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_edl_r_node.set('name', 'edl_r_tendon')
    tendon_edl_r_node.set('springlength', '0.368875')
    sites_edl_r = ['edl_r-P1', 'edl_r-P2', 'edl_r-P3', 'edl_r-P4', 'edl_r-P5', 'edl_r-P6']
    for site in sites_edl_r:
        site_node = ET.SubElement(tendon_edl_r_node, 'site')
        site_node.set('site', site)

    # ehl_r_tendon
    tendon_ehl_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_ehl_r_node.set('name', 'ehl_r_tendon')
    tendon_ehl_r_node.set('springlength', '0.326795')
    sites_ehl_r = ['ehl_r-P1', 'ehl_r-P2', 'ehl_r-P3', 'ehl_r-P4', 'ehl_r-P5', 'ehl_r-P6', 'ehl_r-P7', 'ehl_r-P8']
    for site in sites_ehl_r:
        site_node = ET.SubElement(tendon_ehl_r_node, 'site')
        site_node.set('site', site)

    # fdl_r_tendon
    tendon_fdl_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_fdl_r_node.set('name', 'fdl_r_tendon')
    tendon_fdl_r_node.set('springlength', '0.378773')
    sites_fdl_r = ['fdl_r-P1', 'fdl_r-P2', 'fdl_r-P3', 'fdl_r-P4', 'fdl_r-P5', 'fdl_r-P6', 'fdl_r-P7', 'fdl_r-P8']
    for site in sites_fdl_r:
        site_node = ET.SubElement(tendon_fdl_r_node, 'site')
        site_node.set('site', site)

    # fhl_r_tendon
    tendon_fhl_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_fhl_r_node.set('name', 'fhl_r_tendon')
    tendon_fhl_r_node.set('springlength', '0.35434')
    sites_fhl_r = ['fhl_r-P1', 'fhl_r-P2', 'fhl_r-P3', 'fhl_r-P4', 'fhl_r-P5', 'fhl_r-P6', 'fhl_r-P7']
    for site in sites_fhl_r:
        site_node = ET.SubElement(tendon_fhl_r_node, 'site')
        site_node.set('site', site)

    # gaslat_r_tendon
    tendon_gaslat_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_gaslat_r_node.set('name', 'gaslat_r_tendon')
    tendon_gaslat_r_node.set('springlength', '0.37607')
    sites_gaslat_r = ['gaslat_r-P1', 'gaslat_r-P2']
    for site in sites_gaslat_r:
        site_node = ET.SubElement(tendon_gaslat_r_node, 'site')
        site_node.set('site', site)

    # gasmed_r_tendon
    tendon_gasmed_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_gasmed_r_node.set('name', 'gasmed_r_tendon')
    tendon_gasmed_r_node.set('springlength', '0.398716')
    sites_gasmed_r = ['gasmed_r-P1', 'gasmed_r-P2']
    for site in sites_gasmed_r:
        site_node = ET.SubElement(tendon_gasmed_r_node, 'site')
        site_node.set('site', site)

    # glmax1_r_tendon
    tendon_glmax1_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmax1_r_node.set('name', 'glmax1_r_tendon')
    tendon_glmax1_r_node.set('springlength', '0.0488409')
    sites_glmax1_r = ['glmax1_r-P1', 'glmax1_r-P2', 'glmax1_r-P3', 'glmax1_r-P4']
    for site in sites_glmax1_r:
        site_node = ET.SubElement(tendon_glmax1_r_node, 'site')
        site_node.set('site', site)

    # glmax2_r_tendon
    tendon_glmax2_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmax2_r_node.set('name', 'glmax2_r_tendon')
    tendon_glmax2_r_node.set('springlength', '0.0678872')
    sites_glmax2_r = ['glmax2_r-P1', 'glmax2_r-P2', 'glmax2_r-P3', 'glmax2_r-P4']
    for site in sites_glmax2_r:
        site_node = ET.SubElement(tendon_glmax2_r_node, 'site')
        site_node.set('site', site)

    # glmax3_r_tendon
    tendon_glmax3_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmax3_r_node.set('name', 'glmax3_r_tendon')
    tendon_glmax3_r_node.set('springlength', '0.0697166')
    sites_glmax3_r = ['glmax3_r-P1', 'glmax3_r-P2', 'glmax3_r-P3', 'glmax3_r-P4']
    for site in sites_glmax3_r:
        site_node = ET.SubElement(tendon_glmax3_r_node, 'site')
        site_node.set('site', site)

    # glmed1_r_tendon
    tendon_glmed1_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmed1_r_node.set('name', 'glmed1_r_tendon')
    tendon_glmed1_r_node.set('springlength', '0.0558168')
    sites_glmed1_r = ['glmed1_r-P1', 'glmed1_r-P2']
    for site in sites_glmed1_r:
        site_node = ET.SubElement(tendon_glmed1_r_node, 'site')
        site_node.set('site', site)

    # glmed2_r_tendon
    tendon_glmed2_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmed2_r_node.set('name', 'glmed2_r_tendon')
    tendon_glmed2_r_node.set('springlength', '0.0652488')
    sites_glmed2_r = ['glmed2_r-P1', 'glmed2_r-P2']
    for site in sites_glmed2_r:
        site_node = ET.SubElement(tendon_glmed2_r_node, 'site')
        site_node.set('site', site)

    # glmed3_r_tendon
    tendon_glmed3_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmed3_r_node.set('name', 'glmed3_r_tendon')
    tendon_glmed3_r_node.set('springlength', '0.0452328')
    sites_glmed3_r = ['glmed3_r-P1', 'glmed3_r-P2']
    for site in sites_glmed3_r:
        site_node = ET.SubElement(tendon_glmed3_r_node, 'site')
        site_node.set('site', site)

    # glmin1_r_tendon
    tendon_glmin1_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmin1_r_node.set('name', 'glmin1_r_tendon')
    tendon_glmin1_r_node.set('springlength', '0.0161378')
    sites_glmin1_r = ['glmin1_r-P1', 'glmin1_r-P2']
    for site in sites_glmin1_r:
        site_node = ET.SubElement(tendon_glmin1_r_node, 'site')
        site_node.set('site', site)

    # glmin2_r_tendon
    tendon_glmin2_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmin2_r_node.set('name', 'glmin2_r_tendon')
    tendon_glmin2_r_node.set('springlength', '0.0260991')
    sites_glmin2_r = ['glmin2_r-P1', 'glmin2_r-P2']
    for site in sites_glmin2_r:
        site_node = ET.SubElement(tendon_glmin2_r_node, 'site')
        site_node.set('site', site)

    # glmin3_r_tendon
    tendon_glmin3_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmin3_r_node.set('name', 'glmin3_r_tendon')
    tendon_glmin3_r_node.set('springlength', '0.0508725')
    sites_glmin3_r = ['glmin3_r-P1', 'glmin3_r-P2']
    for site in sites_glmin3_r:
        site_node = ET.SubElement(tendon_glmin3_r_node, 'site')
        site_node.set('site', site)

    # grac_r_tendon
    tendon_grac_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_grac_r_node.set('name', 'grac_r_tendon')
    tendon_grac_r_node.set('springlength', '0.172014')
    sites_grac_r = ['grac_r-P1', 'grac_r-P2', 'grac_r-P3']
    for site in sites_grac_r:
        site_node = ET.SubElement(tendon_grac_r_node, 'site')
        site_node.set('site', site)

    # iliacus_r_tendon
    tendon_iliacus_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_iliacus_r_node.set('name', 'iliacus_r_tendon')
    tendon_iliacus_r_node.set('springlength', '0.0961207')
    sites_iliacus_r = ['iliacus_r-P1', 'iliacus_r-P2', 'iliacus_r-P3', 'iliacus_r-P4']
    for site in sites_iliacus_r:
        site_node = ET.SubElement(tendon_iliacus_r_node, 'site')
        site_node.set('site', site)

    # perbrev_r_tendon
    tendon_perbrev_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_perbrev_r_node.set('name', 'perbrev_r_tendon')
    tendon_perbrev_r_node.set('springlength', '0.147528')
    sites_perbrev_r = ['perbrev_r-P1', 'perbrev_r-P2', 'perbrev_r-P3', 'perbrev_r-P4', 'perbrev_r-P5']
    for site in sites_perbrev_r:
        site_node = ET.SubElement(tendon_perbrev_r_node, 'site')
        site_node.set('site', site)

    # perlong_r_tendon
    tendon_perlong_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_perlong_r_node.set('name', 'perlong_r_tendon')
    tendon_perlong_r_node.set('springlength', '0.332221')
    sites_perlong_r = ['perlong_r-P1', 'perlong_r-P2', 'perlong_r-P3', 'perlong_r-P4', 'perlong_r-P5', 'perlong_r-P6', 'perlong_r-P7']
    for site in sites_perlong_r:
        site_node = ET.SubElement(tendon_perlong_r_node, 'site')
        site_node.set('site', site)

    # piri_r_tendon
    tendon_piri_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_piri_r_node.set('name', 'piri_r_tendon')
    tendon_piri_r_node.set('springlength', '0.114906')
    sites_piri_r = ['piri_r-P1', 'piri_r-P2', 'piri_r-P3']
    for site in sites_piri_r:
        site_node = ET.SubElement(tendon_piri_r_node, 'site')
        site_node.set('site', site)

    # psoas_r_tendon
    tendon_psoas_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_psoas_r_node.set('name', 'psoas_r_tendon')
    tendon_psoas_r_node.set('springlength', '0.0995432')
    sites_psoas_r = ['psoas_r-P1', 'psoas_r-P2', 'psoas_r-P3', 'psoas_r-P4']
    for site in sites_psoas_r:
        site_node = ET.SubElement(tendon_psoas_r_node, 'site')
        site_node.set('site', site)

    # recfem_r_tendon
    tendon_recfem_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_recfem_r_node.set('name', 'recfem_r_tendon')
    tendon_recfem_r_node.set('springlength', '0.448494')
    sites_recfem_r = ['recfem_r-P1', 'recfem_r-P2', 'recfem_r-P3', 'recfem_r-P4', 'recfem_r-P5']
    for site in sites_recfem_r:
        site_node = ET.SubElement(tendon_recfem_r_node, 'site')
        site_node.set('site', site)

    # sart_r_tendon
    tendon_sart_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_sart_r_node.set('name', 'sart_r_tendon')
    tendon_sart_r_node.set('springlength', '0.124')
    sites_sart_r = ['sart_r-P1', 'sart_r-P2', 'sart_r-P3', 'sart_r-P4', 'sart_r-P5']
    for site in sites_sart_r:
        site_node = ET.SubElement(tendon_sart_r_node, 'site')
        site_node.set('site', site)

    # semimem_r_tendon
    tendon_semimem_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_semimem_r_node.set('name', 'semimem_r_tendon')
    tendon_semimem_r_node.set('springlength', '0.347586')
    sites_semimem_r = ['semimem_r-P1', 'semimem_r-P2']
    for site in sites_semimem_r:
        site_node = ET.SubElement(tendon_semimem_r_node, 'site')
        site_node.set('site', site)

    # semiten_r_tendon
    tendon_semiten_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_semiten_r_node.set('name', 'semiten_r_tendon')
    tendon_semiten_r_node.set('springlength', '0.247199')
    sites_semiten_r = ['semiten_r-P1', 'semiten_r-P2', 'semiten_r-P3']
    for site in sites_semiten_r:
        site_node = ET.SubElement(tendon_semiten_r_node, 'site')
        site_node.set('site', site)

    # soleus_r_tendon
    tendon_soleus_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_soleus_r_node.set('name', 'soleus_r_tendon')
    tendon_soleus_r_node.set('springlength', '0.276756')
    sites_soleus_r = ['soleus_r-P1', 'soleus_r-P2']
    for site in sites_soleus_r:
        site_node = ET.SubElement(tendon_soleus_r_node, 'site')
        site_node.set('site', site)

    # tfl_r_tendon
    tendon_tfl_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_tfl_r_node.set('name', 'tfl_r_tendon')
    tendon_tfl_r_node.set('springlength', '0.449497')
    sites_tfl_r = ['tfl_r-P1', 'tfl_r-P2', 'tfl_r-P3', 'tfl_r-P4']
    for site in sites_tfl_r:
        site_node = ET.SubElement(tendon_tfl_r_node, 'site')
        site_node.set('site', site)

    # tibant_r_tendon
    tendon_tibant_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_tibant_r_node.set('name', 'tibant_r_tendon')
    tendon_tibant_r_node.set('springlength', '0.240461')
    sites_tibant_r = ['tibant_r-P1', 'tibant_r-P2', 'tibant_r-P3', 'tibant_r-P4']
    for site in sites_tibant_r:
        site_node = ET.SubElement(tendon_tibant_r_node, 'site')
        site_node.set('site', site)

    # tibpost_r_tendon
    tendon_tibpost_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_tibpost_r_node.set('name', 'tibpost_r_tendon')
    tendon_tibpost_r_node.set('springlength', '0.28078')
    sites_tibpost_r = ['tibpost_r-P1', 'tibpost_r-P2', 'tibpost_r-P3', 'tibpost_r-P4']
    for site in sites_tibpost_r:
        site_node = ET.SubElement(tendon_tibpost_r_node, 'site')
        site_node.set('site', site)

    # vasint_r_tendon
    tendon_vasint_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_vasint_r_node.set('name', 'vasint_r_tendon')
    tendon_vasint_r_node.set('springlength', '0.202211')
    sites_vasint_r = ['vasint_r-P1', 'vasint_r-P2', 'vasint_r-P3', 'vasint_r-P4', 'vasint_r-P5']
    for site in sites_vasint_r:
        site_node = ET.SubElement(tendon_vasint_r_node, 'site')
        site_node.set('site', site)

    # vaslat_r_tendon
    tendon_vaslat_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_vaslat_r_node.set('name', 'vaslat_r_tendon')
    tendon_vaslat_r_node.set('springlength', '0.220601')
    sites_vaslat_r = ['vaslat_r-P1', 'vaslat_r-P2', 'vaslat_r-P3', 'vaslat_r-P4', 'vaslat_r-P5']
    for site in sites_vaslat_r:
        site_node = ET.SubElement(tendon_vaslat_r_node, 'site')
        site_node.set('site', site)

    # vasmed_r_tendon
    tendon_vasmed_r_node = ET.SubElement(tendon_node, 'spatial')
    tendon_vasmed_r_node.set('name', 'vasmed_r_tendon')
    tendon_vasmed_r_node.set('springlength', '0.199904')
    sites_vasmed_r = ['vasmed_r-P1', 'vasmed_r-P2', 'vasmed_r-P3', 'vasmed_r-P4', 'vasmed_r-P5']
    for site in sites_vasmed_r:
        site_node = ET.SubElement(tendon_vasmed_r_node, 'site')
        site_node.set('site', site)

    # addbrev_l_tendon
    tendon_addbrev_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addbrev_l_node.set('name', 'addbrev_l_tendon')
    tendon_addbrev_l_node.set('springlength', '0.0354503')
    sites_addbrev_l = ['addbrev_l-P1', 'addbrev_l-P2']
    for site in sites_addbrev_l:
        site_node = ET.SubElement(tendon_addbrev_l_node, 'site')
        site_node.set('site', site)

    # addlong_l_tendon
    tendon_addlong_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addlong_l_node.set('name', 'addlong_l_tendon')
    tendon_addlong_l_node.set('springlength', '0.131799')
    sites_addlong_l = ['addlong_l-P1', 'addlong_l-P2']
    for site in sites_addlong_l:
        site_node = ET.SubElement(tendon_addlong_l_node, 'site')
        site_node.set('site', site)

    # addmagDist_l_tendon
    tendon_addmagDist_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addmagDist_l_node.set('name', 'addmagDist_l_tendon')
    tendon_addmagDist_l_node.set('springlength', '0.0873807')
    sites_addmagDist_l = ['addmagDist_l-P1', 'addmagDist_l-P2']
    for site in sites_addmagDist_l:
        site_node = ET.SubElement(tendon_addmagDist_l_node, 'site')
        site_node.set('site', site)

    # addmagIsch_l_tendon
    tendon_addmagIsch_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addmagIsch_l_node.set('name', 'addmagIsch_l_tendon')
    tendon_addmagIsch_l_node.set('springlength', '0.216343')
    sites_addmagIsch_l = ['addmagIsch_l-P1', 'addmagIsch_l-P2']
    for site in sites_addmagIsch_l:
        site_node = ET.SubElement(tendon_addmagIsch_l_node, 'site')
        site_node.set('site', site)

    # addmagMid_l_tendon
    tendon_addmagMid_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addmagMid_l_node.set('name', 'addmagMid_l_tendon')
    tendon_addmagMid_l_node.set('springlength', '0.0466277')
    sites_addmagMid_l = ['addmagMid_l-P1', 'addmagMid_l-P2']
    for site in sites_addmagMid_l:
        site_node = ET.SubElement(tendon_addmagMid_l_node, 'site')
        site_node.set('site', site)

    # addmagProx_l_tendon
    tendon_addmagProx_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_addmagProx_l_node.set('name', 'addmagProx_l_tendon')
    tendon_addmagProx_l_node.set('springlength', '0.0403241')
    sites_addmagProx_l = ['addmagProx_l-P1', 'addmagProx_l-P2']
    for site in sites_addmagProx_l:
        site_node = ET.SubElement(tendon_addmagProx_l_node, 'site')
        site_node.set('site', site)

    # bflh_l_tendon
    tendon_bflh_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_bflh_l_node.set('name', 'bflh_l_tendon')
    tendon_bflh_l_node.set('springlength', '0.325227')
    sites_bflh_l = ['bflh_l-P1', 'bflh_l-P2', 'bflh_l-P3']
    for site in sites_bflh_l:
        site_node = ET.SubElement(tendon_bflh_l_node, 'site')
        site_node.set('site', site)

    # bfsh_l_tendon
    tendon_bfsh_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_bfsh_l_node.set('name', 'bfsh_l_tendon')
    tendon_bfsh_l_node.set('springlength', '0.105817')
    sites_bfsh_l = ['bfsh_l-P1', 'bfsh_l-P2']
    for site in sites_bfsh_l:
        site_node = ET.SubElement(tendon_bfsh_l_node, 'site')
        site_node.set('site', site)

    # edl_l_tendon
    tendon_edl_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_edl_l_node.set('name', 'edl_l_tendon')
    tendon_edl_l_node.set('springlength', '0.368875')
    sites_edl_l = ['edl_l-P1', 'edl_l-P2', 'edl_l-P3', 'edl_l-P4', 'edl_l-P5', 'edl_l-P6']
    for site in sites_edl_l:
        site_node = ET.SubElement(tendon_edl_l_node, 'site')
        site_node.set('site', site)

    # ehl_l_tendon
    tendon_ehl_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_ehl_l_node.set('name', 'ehl_l_tendon')
    tendon_ehl_l_node.set('springlength', '0.326795')
    sites_ehl_l = ['ehl_l-P1', 'ehl_l-P2', 'ehl_l-P3', 'ehl_l-P4', 'ehl_l-P5', 'ehl_l-P6', 'ehl_l-P7', 'ehl_l-P8']
    for site in sites_ehl_l:
        site_node = ET.SubElement(tendon_ehl_l_node, 'site')
        site_node.set('site', site)

    # fdl_l_tendon
    tendon_fdl_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_fdl_l_node.set('name', 'fdl_l_tendon')
    tendon_fdl_l_node.set('springlength', '0.378773')
    sites_fdl_l = ['fdl_l-P1', 'fdl_l-P2', 'fdl_l-P3', 'fdl_l-P4', 'fdl_l-P5', 'fdl_l-P6', 'fdl_l-P7', 'fdl_l-P8']
    for site in sites_fdl_l:
        site_node = ET.SubElement(tendon_fdl_l_node, 'site')
        site_node.set('site', site)

    # fhl_l_tendon
    tendon_fhl_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_fhl_l_node.set('name', 'fhl_l_tendon')
    tendon_fhl_l_node.set('springlength', '0.35434')
    sites_fhl_l = ['fhl_l-P1', 'fhl_l-P2', 'fhl_l-P3', 'fhl_l-P4', 'fhl_l-P5', 'fhl_l-P6', 'fhl_l-P7']
    for site in sites_fhl_l:
        site_node = ET.SubElement(tendon_fhl_l_node, 'site')
        site_node.set('site', site)

    # gaslat_l_tendon
    tendon_gaslat_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_gaslat_l_node.set('name', 'gaslat_l_tendon')
    tendon_gaslat_l_node.set('springlength', '0.37607')
    sites_gaslat_l = ['gaslat_l-P1', 'gaslat_l-P2']
    for site in sites_gaslat_l:
        site_node = ET.SubElement(tendon_gaslat_l_node, 'site')
        site_node.set('site', site)

    # gasmed_l_tendon
    tendon_gasmed_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_gasmed_l_node.set('name', 'gasmed_l_tendon')
    tendon_gasmed_l_node.set('springlength', '0.398716')
    sites_gasmed_l = ['gasmed_l-P1', 'gasmed_l-P2']
    for site in sites_gasmed_l:
        site_node = ET.SubElement(tendon_gasmed_l_node, 'site')
        site_node.set('site', site)

    # glmax1_l_tendon
    tendon_glmax1_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmax1_l_node.set('name', 'glmax1_l_tendon')
    tendon_glmax1_l_node.set('springlength', '0.0488409')
    sites_glmax1_l = ['glmax1_l-P1', 'glmax1_l-P2', 'glmax1_l-P3', 'glmax1_l-P4']
    for site in sites_glmax1_l:
        site_node = ET.SubElement(tendon_glmax1_l_node, 'site')
        site_node.set('site', site)

    # glmax2_l_tendon
    tendon_glmax2_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmax2_l_node.set('name', 'glmax2_l_tendon')
    tendon_glmax2_l_node.set('springlength', '0.0678872')
    sites_glmax2_l = ['glmax2_l-P1', 'glmax2_l-P2', 'glmax2_l-P3', 'glmax2_l-P4']
    for site in sites_glmax2_l:
        site_node = ET.SubElement(tendon_glmax2_l_node, 'site')
        site_node.set('site', site)

    # glmax3_l_tendon
    tendon_glmax3_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmax3_l_node.set('name', 'glmax3_l_tendon')
    tendon_glmax3_l_node.set('springlength', '0.0697166')
    sites_glmax3_l = ['glmax3_l-P1', 'glmax3_l-P2', 'glmax3_l-P3', 'glmax3_l-P4']
    for site in sites_glmax3_l:
        site_node = ET.SubElement(tendon_glmax3_l_node, 'site')
        site_node.set('site', site)

    # glmed1_l_tendon
    tendon_glmed1_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmed1_l_node.set('name', 'glmed1_l_tendon')
    tendon_glmed1_l_node.set('springlength', '0.0558168')
    sites_glmed1_l = ['glmed1_l-P1', 'glmed1_l-P2']
    for site in sites_glmed1_l:
        site_node = ET.SubElement(tendon_glmed1_l_node, 'site')
        site_node.set('site', site)

    # glmed2_l_tendon
    tendon_glmed2_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmed2_l_node.set('name', 'glmed2_l_tendon')
    tendon_glmed2_l_node.set('springlength', '0.0652488')
    sites_glmed2_l = ['glmed2_l-P1', 'glmed2_l-P2']
    for site in sites_glmed2_l:
        site_node = ET.SubElement(tendon_glmed2_l_node, 'site')
        site_node.set('site', site)

    # glmed3_l_tendon
    tendon_glmed3_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmed3_l_node.set('name', 'glmed3_l_tendon')
    tendon_glmed3_l_node.set('springlength', '0.0452328')
    sites_glmed3_l = ['glmed3_l-P1', 'glmed3_l-P2']
    for site in sites_glmed3_l:
        site_node = ET.SubElement(tendon_glmed3_l_node, 'site')
        site_node.set('site', site)

    # glmin1_l_tendon
    tendon_glmin1_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmin1_l_node.set('name', 'glmin1_l_tendon')
    tendon_glmin1_l_node.set('springlength', '0.0161378')
    sites_glmin1_l = ['glmin1_l-P1', 'glmin1_l-P2']
    for site in sites_glmin1_l:
        site_node = ET.SubElement(tendon_glmin1_l_node, 'site')
        site_node.set('site', site)

    # glmin2_l_tendon
    tendon_glmin2_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmin2_l_node.set('name', 'glmin2_l_tendon')
    tendon_glmin2_l_node.set('springlength', '0.0260991')
    sites_glmin2_l = ['glmin2_l-P1', 'glmin2_l-P2']
    for site in sites_glmin2_l:
        site_node = ET.SubElement(tendon_glmin2_l_node, 'site')
        site_node.set('site', site)

    # glmin3_l_tendon
    tendon_glmin3_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_glmin3_l_node.set('name', 'glmin3_l_tendon')
    tendon_glmin3_l_node.set('springlength', '0.0508725')
    sites_glmin3_l = ['glmin3_l-P1', 'glmin3_l-P2']
    for site in sites_glmin3_l:
        site_node = ET.SubElement(tendon_glmin3_l_node, 'site')
        site_node.set('site', site)

    # grac_l_tendon
    tendon_grac_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_grac_l_node.set('name', 'grac_l_tendon')
    tendon_grac_l_node.set('springlength', '0.172014')
    sites_grac_l = ['grac_l-P1', 'grac_l-P2', 'grac_l-P3']
    for site in sites_grac_l:
        site_node = ET.SubElement(tendon_grac_l_node, 'site')
        site_node.set('site', site)

    # iliacus_l_tendon
    tendon_iliacus_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_iliacus_l_node.set('name', 'iliacus_l_tendon')
    tendon_iliacus_l_node.set('springlength', '0.0961207')
    sites_iliacus_l = ['iliacus_l-P1', 'iliacus_l-P2', 'iliacus_l-P3', 'iliacus_l-P4']
    for site in sites_iliacus_l:
        site_node = ET.SubElement(tendon_iliacus_l_node, 'site')
        site_node.set('site', site)

    # perbrev_l_tendon
    tendon_perbrev_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_perbrev_l_node.set('name', 'perbrev_l_tendon')
    tendon_perbrev_l_node.set('springlength', '0.147528')
    sites_perbrev_l = ['perbrev_l-P1', 'perbrev_l-P2', 'perbrev_l-P3', 'perbrev_l-P4', 'perbrev_l-P5']
    for site in sites_perbrev_l:
        site_node = ET.SubElement(tendon_perbrev_l_node, 'site')
        site_node.set('site', site)

    # perlong_l_tendon
    tendon_perlong_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_perlong_l_node.set('name', 'perlong_l_tendon')
    tendon_perlong_l_node.set('springlength', '0.332221')
    sites_perlong_l = ['perlong_l-P1', 'perlong_l-P2', 'perlong_l-P3', 'perlong_l-P4', 'perlong_l-P5', 'perlong_l-P6', 'perlong_l-P7']
    for site in sites_perlong_l:
        site_node = ET.SubElement(tendon_perlong_l_node, 'site')
        site_node.set('site', site)

    # piri_l_tendon
    tendon_piri_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_piri_l_node.set('name', 'piri_l_tendon')
    tendon_piri_l_node.set('springlength', '0.114906')
    sites_piri_l = ['piri_l-P1', 'piri_l-P2', 'piri_l-P3']
    for site in sites_piri_l:
        site_node = ET.SubElement(tendon_piri_l_node, 'site')
        site_node.set('site', site)

    # psoas_l_tendon
    tendon_psoas_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_psoas_l_node.set('name', 'psoas_l_tendon')
    tendon_psoas_l_node.set('springlength', '0.0995432')
    sites_psoas_l = ['psoas_l-P1', 'psoas_l-P2', 'psoas_l-P3', 'psoas_l-P4']
    for site in sites_psoas_l:
        site_node = ET.SubElement(tendon_psoas_l_node, 'site')
        site_node.set('site', site)

    # recfem_l_tendon
    tendon_recfem_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_recfem_l_node.set('name', 'recfem_l_tendon')
    tendon_recfem_l_node.set('springlength', '0.448494')
    sites_recfem_l = ['recfem_l-P1', 'recfem_l-P2', 'recfem_l-P3', 'recfem_l-P4', 'recfem_l-P5']
    for site in sites_recfem_l:
        site_node = ET.SubElement(tendon_recfem_l_node, 'site')
        site_node.set('site', site)

    # sart_l_tendon
    tendon_sart_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_sart_l_node.set('name', 'sart_l_tendon')
    tendon_sart_l_node.set('springlength', '0.124')
    sites_sart_l = ['sart_l-P1', 'sart_l-P2', 'sart_l-P3', 'sart_l-P4', 'sart_l-P5']
    for site in sites_sart_l:
        site_node = ET.SubElement(tendon_sart_l_node, 'site')
        site_node.set('site', site)

    # semimem_l_tendon
    tendon_semimem_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_semimem_l_node.set('name', 'semimem_l_tendon')
    tendon_semimem_l_node.set('springlength', '0.347586')
    sites_semimem_l = ['semimem_l-P1', 'semimem_l-P2']
    for site in sites_semimem_l:
        site_node = ET.SubElement(tendon_semimem_l_node, 'site')
        site_node.set('site', site)

    # semiten_l_tendon
    tendon_semiten_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_semiten_l_node.set('name', 'semiten_l_tendon')
    tendon_semiten_l_node.set('springlength', '0.247199')
    sites_semiten_l = ['semiten_l-P1', 'semiten_l-P2', 'semiten_l-P3']
    for site in sites_semiten_l:
        site_node = ET.SubElement(tendon_semiten_l_node, 'site')
        site_node.set('site', site)

    # soleus_l_tendon
    tendon_soleus_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_soleus_l_node.set('name', 'soleus_l_tendon')
    tendon_soleus_l_node.set('springlength', '0.276756')
    sites_soleus_l = ['soleus_l-P1', 'soleus_l-P2']
    for site in sites_soleus_l:
        site_node = ET.SubElement(tendon_soleus_l_node, 'site')
        site_node.set('site', site)

    # tfl_l_tendon
    tendon_tfl_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_tfl_l_node.set('name', 'tfl_l_tendon')
    tendon_tfl_l_node.set('springlength', '0.449497')
    sites_tfl_l = ['tfl_l-P1', 'tfl_l-P2', 'tfl_l-P3', 'tfl_l-P4']
    for site in sites_tfl_l:
        site_node = ET.SubElement(tendon_tfl_l_node, 'site')
        site_node.set('site', site)

    # tibant_l_tendon
    tendon_tibant_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_tibant_l_node.set('name', 'tibant_l_tendon')
    tendon_tibant_l_node.set('springlength', '0.240461')
    sites_tibant_l = ['tibant_l-P1', 'tibant_l-P2', 'tibant_l-P3', 'tibant_l-P4']
    for site in sites_tibant_l:
        site_node = ET.SubElement(tendon_tibant_l_node, 'site')
        site_node.set('site', site)

    # tibpost_l_tendon
    tendon_tibpost_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_tibpost_l_node.set('name', 'tibpost_l_tendon')
    tendon_tibpost_l_node.set('springlength', '0.28078')
    sites_tibpost_l = ['tibpost_l-P1', 'tibpost_l-P2', 'tibpost_l-P3', 'tibpost_l-P4']
    for site in sites_tibpost_l:
        site_node = ET.SubElement(tendon_tibpost_l_node, 'site')
        site_node.set('site', site)

    # vasint_l_tendon
    tendon_vasint_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_vasint_l_node.set('name', 'vasint_l_tendon')
    tendon_vasint_l_node.set('springlength', '0.202211')
    sites_vasint_l = ['vasint_l-P1', 'vasint_l-P2', 'vasint_l-P3', 'vasint_l-P4', 'vasint_l-P5']
    for site in sites_vasint_l:
        site_node = ET.SubElement(tendon_vasint_l_node, 'site')
        site_node.set('site', site)

    # vaslat_l_tendon
    tendon_vaslat_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_vaslat_l_node.set('name', 'vaslat_l_tendon')
    tendon_vaslat_l_node.set('springlength', '0.220601')
    sites_vaslat_l = ['vaslat_l-P1', 'vaslat_l-P2', 'vaslat_l-P3', 'vaslat_l-P4', 'vaslat_l-P5']
    for site in sites_vaslat_l:
        site_node = ET.SubElement(tendon_vaslat_l_node, 'site')
        site_node.set('site', site)

    # vasmed_l_tendon
    tendon_vasmed_l_node = ET.SubElement(tendon_node, 'spatial')
    tendon_vasmed_l_node.set('name', 'vasmed_l_tendon')
    tendon_vasmed_l_node.set('springlength', '0.199904')
    sites_vasmed_l = ['vasmed_l-P1', 'vasmed_l-P2', 'vasmed_l-P3', 'vasmed_l-P4', 'vasmed_l-P5']
    for site in sites_vasmed_l:
        site_node = ET.SubElement(tendon_vasmed_l_node, 'site')
        site_node.set('site', site)

    #Contact
    contact_node = ET.SubElement(root,'contact')
    contact_pair1 = ET.SubElement(contact_node, 'pair')
    contact_pair1.set('condim','4')
    contact_pair1.set('friction','1 1 0.005 0.0001 0.0001')
    contact_pair1.set('geom1','left_sole')
    contact_pair1.set('geom2','plane')
    contact_pair1.set('margin','0.0015')

    contact_pair2 = ET.SubElement(contact_node, 'pair')
    contact_pair2.set('condim','4')
    contact_pair2.set('friction','1 1 0.005 0.0001 0.0001')
    contact_pair2.set('geom1','right_sole')
    contact_pair2.set('geom2','plane')
    contact_pair2.set('margin','0.0015')

    contact_pair3 = ET.SubElement(contact_node, 'pair')
    contact_pair3.set('condim','4')
    contact_pair3.set('friction','1 1 0.005 0.0001 0.0001')
    contact_pair3.set('geom1','left_toe')
    contact_pair3.set('geom2','plane')
    contact_pair3.set('margin','0.0015')

    contact_pair4 = ET.SubElement(contact_node, 'pair')
    contact_pair4.set('condim','4')
    contact_pair4.set('friction','1 1 0.005 0.0001 0.0001')
    contact_pair4.set('geom1','right_toe')
    contact_pair4.set('geom2','plane')
    contact_pair4.set('margin','0.0015')

    contact_pair5 = ET.SubElement(contact_node, 'pair')
    contact_pair5.set('condim','4')
    contact_pair5.set('friction','1 1 0.005 0.0001 0.0001')
    contact_pair5.set('geom1','left_heel')
    contact_pair5.set('geom2','plane')
    contact_pair5.set('margin','0.0015')

    contact_pair6 = ET.SubElement(contact_node, 'pair')
    contact_pair6.set('condim','4')
    contact_pair6.set('friction','1 1 0.005 0.0001 0.0001')
    contact_pair6.set('geom1','right_heel')
    contact_pair6.set('geom2','plane')
    contact_pair6.set('margin','0.0015')


    #Actuators
    actuator_node = ET.SubElement(root, 'actuator')
      #Creating the 'general' elements
    general_actuators = [
        {
            'biasprm': "0.381903 1.46199 604.625 1 0.0669801 2.99607 10 1.34206 1.4 0",
            'class': "muscle",
            'gainprm': "0.381903 1.46199 604.625 1 0.0669801 2.99607 10 1.34206 1.4 0",
            'lengthrange': "0.0748245 0.186181",
            'name': "addbrev_r",
            'tendon': "addbrev_r_tendon"
        },
        {
            'biasprm': "0.361607 1.56839 881.938 1 0.126715 3.5 10 1.50984 1.4 0",
            'class': "muscle",
            'gainprm': "0.361607 1.56839 881.938 1 0.126715 3.5 10 1.50984 1.4 0",
            'lengthrange': "0.170925 0.301499",
            'name': "addlong_r",
            'tendon': "addlong_r_tendon"
        },
        {
            'biasprm': "0.647072 1.32551 576.186 1 0.000543367 2.1578 10 1.60782 1.4 0",
            'class': "muscle",
            'gainprm': "0.647072 1.32551 576.186 1 0.000543367 2.1578 10 1.60782 1.4 0",
            'lengthrange': "0.202042 0.322261",
            'name': "addmagDist_r",
            'tendon': "addmagDist_r_tendon"
        },
        {
            'biasprm': "0.662907 1.54717 564.689 1 0.132471 3.5 10 1.6643 1.4 0",
            'class': "muscle",
            'gainprm': "0.662907 1.54717 564.689 1 0.132471 3.5 10 1.6643 1.4 0",
            'lengthrange': "0.319889 0.458011",
            'name': "addmagIsch_r",
            'tendon': "addmagIsch_r_tendon"
        },
        # Add the rest of the 'general' elements similarly...
    ]

    for actuator in general_actuators:
        general_node = ET.SubElement(actuator_node, 'general')
        for key, value in actuator.items():
            general_node.set(key, value)

    # Creating the 'motor' elements
    motor_actuators = [
        {
            'ctrllimited': "true",
            'ctrlrange': "-350  350",
            'joint': "LeftFrontalHipJoint",
            'name': "LeftFrontalHipLink_actuator"
        },
        {
            'ctrllimited': "true",
            'ctrlrange': "-180  180",
            'joint': "LeftTransverseHipJoint",
            'name': "LeftTransverseHipLink_actuator"
        },
        {
            'ctrllimited': "true",
            'ctrlrange': "-219  219",
            'joint': "LeftSagittalHipJoint",
            'name': "LeftSagittalHipLink_actuator"
        },
        {
            'ctrllimited': "true",
            'ctrlrange': "-219  219",
            'joint': "LeftSagittalKneeJoint",
            'name': "LeftSagittalKneeLink_actuator"
        },
        {
            'ctrllimited': "true",
            'ctrlrange': "-184  184",
            'joint': "LeftSagittalAnkleJoint",
            'name': "LeftSagittalAnkleLink_actuator"
        },
        {
            'ctrllimited': "true",
            'ctrlrange': "-82  82",
            'joint': "LeftHenkeAnkleJoint",
            'name': "LeftHenkeAnkleLink_actuator"
        },
        {
            'ctrllimited': "true",
            'ctrlrange': "-350  350",
            'joint': "RightFrontalHipJoint",
            'name': "RightFrontalHipLink_actuator"
        },
        {
            'ctrllimited': "true",
            'ctrlrange': "-180  180",
            'joint': "RightTransverseHipJoint",
            'name': "RightTransverseHipLink_actuator"
        },
        {
            'ctrllimited': "true",
            'ctrlrange': "-219  219",
            'joint': "RightSagittalHipJoint",
            'name': "RightSagittalHipLink_actuator"
        },
        {
            'ctrllimited': "true",
            'ctrlrange': "-219  219",
            'joint': "RightSagittalKneeJoint",
            'name': "RightSagittalKneeLink_actuator"
        },
        {
            'ctrllimited': "true",
            'ctrlrange': "-184  184",
            'joint': "RightSagittalAnkleJoint",
            'name': "RightSagittalAnkleLink_actuator"
        },
        {
            'ctrllimited': "true",
            'ctrlrange': "-82  82",
            'joint': "RightHenkeAnkleJoint",
            'name': "RightHenkeAnkleLink_actuator"
        },
    ]

    for motor in motor_actuators:
        motor_node = ET.SubElement(actuator_node, 'motor')
        for key, value in motor.items():
            motor_node.set(key, value)

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


    xml_str = ET.tostring(root, encoding="unicode", method="xml")
    xml_str = xml.dom.minidom.parseString(xml_str).toprettyxml(indent="  ", newl="\n")

    with open("emptymujoco_model.xml", "w") as f:
        f.write(xml_str)