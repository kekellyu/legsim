import xml.etree.ElementTree as ET
import xml.dom.minidom

def parse_existing_xml(emptywmuscle):
    tree = ET.parse(emptywmuscle.xml)
    root = tree.getroot()
    worldbody_element = root.find('worldbody')  # Assuming 'worldbody' is a direct child of the root
    return worldbody_element

# tree = ET.parse('emptywmuscle.xml')
root = ET.Element("mujoco_model")
root.set('mujoco','Amy_Li_emptyExo')

compiler_node = ET.SubElement(root,'compiler')
compiler_node.set('angle', 'radian')
compiler_node.set('autolimites', 'true')

size_node = ET.SubElement(root,'size')
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

mujoco_option = ET.SubElement(root,'option')
mujoco_option.set('timestep', '0.001')
mujoco_option.set('iterations', '50')
mujoco_option.set('solver', 'Newton')
mujoco_option.set('gravity', '0 0 -9.81')
mujoco_option.set('integrator', 'RK4')

mujoco_visual = ET.SubElement(root,'visual')
# root.insert(list(root).index(root.find('asset')), mujoco_visual)
headlight = ET.SubElement(mujoco_visual, 'headlight')
headlight.set('ambient', '0.5 0.5 0.5')
map_node = ET.SubElement(mujoco_visual, 'map')
map_node.set('force', '0.005')
rgba_node = ET.SubElement(mujoco_visual, 'rgba')
rgba_node.set('contactforce', '0.7 0.9 0.9 0.5')


asset_node = ET.SubElement(root, 'asset')

# List of mesh files
mesh_files = [
    "amber_sim/EMPTY_STL/PelvisLink.STL",
    "amber_sim/EMPTY_STL/LeftFrontalHipLink.STL",
    "amber_sim/EMPTY_STL/LeftTransverseHipLink.STL",
    "amber_sim/EMPTY_STL/LeftSagittalHipLink.STL",
    "amber_sim/EMPTY_STL/LeftSagittalKneeLink.STL",
    "amber_sim/EMPTY_STL/LeftSagittalAnkleLink.STL",
    "amber_sim/EMPTY_STL/LeftHenkeAnkleLink.STL",
    "amber_sim/EMPTY_STL/RightFrontalHipLink.STL",
    "amber_sim/EMPTY_STL/RightTransverseHipLink.STL",
    "amber_sim/EMPTY_STL/RightSagittalHipLink.STL",
    "amber_sim/EMPTY_STL/RightSagittalKneeLink.STL",
    "amber_sim/EMPTY_STL/RightSagittalAnkleLink.STL",
    "amber_sim/EMPTY_STL/RightHenkeAnkleLink.STL"
]

# Add mesh elements to the asset node
for mesh_file in mesh_files:
    mesh = ET.SubElement(asset_node, 'mesh')
    mesh.set('file', mesh_file)
    mesh.set('name', mesh_file.split("/")[-1].split(".")[0])  # Extracting name from file path

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

texture_node = ET.SubElement(asset_node, 'texture')
texture_node.set('name', 'tex_myolegs')
texture_node.set('builtin', 'flat')
texture_node.set('height', '762')
texture_node.set('mark', 'cross')
texture_node.set('markrgb', '1 .9 .9')
texture_node.set('rgb1', '1 1 1')
texture_node.set('rgb2', '1 1 1')
texture_node.set('type', 'cube')
texture_node.set('width', '127')

# Add material element referencing the texture
material_node = ET.SubElement(asset_node, 'material')
material_node.set('name', 'mat_myolegs')
material_node.set('texture', 'tex_myolegs')
material_node.set('texuniform', 'true')

# List of mesh files
mesh_files_myo = [
    {"file": "myo_sim/meshes/r_pelvis.stl", "name": "r_pelvis"},
    {"file": "myo_sim/meshes/l_pelvis.stl", "name": "l_pelvis"},
    {"file": "myo_sim/meshes/sacrum.stl", "name": "sacrum"},
    {"file": "myo_sim/meshes/r_femur.stl", "name": "r_femur"},
    {"file": "myo_sim/meshes/r_tibia.stl", "name": "r_tibia"},
    {"file": "myo_sim/meshes/r_fibula.stl", "name": "r_fibula"},
    {"file": "myo_sim/meshes/r_talus.stl", "name": "r_talus"},
    {"file": "myo_sim/meshes/r_foot.stl", "name": "r_foot"},
    {"file": "myo_sim/meshes/r_bofoot.stl", "name": "r_bofoot"},
    {"file": "myo_sim/meshes/r_patella.stl", "name": "r_patella"},
    {"file": "myo_sim/meshes/l_femur.stl", "name": "l_femur"},
    {"file": "myo_sim/meshes/l_tibia.stl", "name": "l_tibia"},
    {"file": "myo_sim/meshes/l_fibula.stl", "name": "l_fibula"},
    {"file": "myo_sim/meshes/l_talus.stl", "name": "l_talus"},
    {"file": "myo_sim/meshes/l_foot.stl", "name": "l_foot"},
    {"file": "myo_sim/meshes/l_bofoot.stl", "name": "l_bofoot"},
    {"file": "myo_sim/meshes/l_patella.stl", "name": "l_patella"}
]
for mesh_info in mesh_files_myo:
    mesh = ET.SubElement(asset_node, 'mesh')
    mesh.set('file', mesh_info['file'])
    mesh.set('name', mesh_info['name'])






# Tendon
tendon_node = ET.SubElement(root,'tendon')

tendon_righthip_node = ET.SubElement(tendon_node,'spatial')
tendon_righthip_node.set('name','righthipconnect')
tendon_righthip_node.set('limited','true')
tendon_righthip_node.set('range','0 0.5')
tendon_righthip_node.set('rgba','0 0 0 0.5')
tendon_righthip_node.set('width','.005')
tendon_righthip_site1= ET.SubElement(tendon_righthip_node,'site')
tendon_righthip_site1.set('site','RightSagittalHipSite')
tendon_righthip_site2= ET.SubElement(tendon_righthip_node,'site')
tendon_righthip_site2.set('site','hip_r_location')
tendon_righthip_site3= ET.SubElement(tendon_righthip_node,'site')
tendon_righthip_site3.set('site','RightTransverseHipSite')

tendon_rightknee_node = ET.SubElement(tendon_node,'spatial')
tendon_rightknee_node.set('name','rightkneeconnect')
tendon_rightknee_node.set('limited','true')
tendon_rightknee_node.set('range','0 0.45')
tendon_rightknee_node.set('rgba','0 0 0 0.5')
tendon_rightknee_node.set('width','.005')
tendon_rightknee_site1=ET.SubElement(tendon_rightknee_node,'site')
tendon_rightknee_site1.set('site','RightSagittalKneeFront')
tendon_rightknee_site2= ET.SubElement(tendon_rightknee_node,'site')
tendon_rightknee_site2.set('site','RightKneeFront')
tendon_rightknee_site3= ET.SubElement(tendon_rightknee_node,'site')
tendon_rightknee_site3.set('site','knee_r_location')
tendon_rightknee_site4= ET.SubElement(tendon_rightknee_node,'site')
tendon_rightknee_site4.set('site','RightKneeBack')
tendon_rightknee_site5= ET.SubElement(tendon_rightknee_node,'site')
tendon_rightknee_site5.set('site','RightSagittalKneeBack')

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

#Worldbody
emptywmuscle_path = 'emptywmuscle.xml'
tree = ET.parse(emptywmuscle_path)
emptywmuscle_root = tree.getroot()
worldbody_element = emptywmuscle_root.find('worldbody')
worldbody_new = ET.SubElement(root,'worldbody')  
worldbody_new.append(worldbody_element) 

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
    {
        'biasprm': "0.477896 1.27927 577.178 1 0.0269359 2.05759 10 1.69396 1.4 0",
        'class': "muscle",
        'gainprm': "0.477896 1.27927 577.178 1 0.0269359 2.05759 10 1.69396 1.4 0",
        'lengthrange': "0.112434 0.222784",
        'name': "addmagMid_r",
        'tendon': "addmagMid_r_tendon"
    },
    {
        'biasprm': "0.1628 1.1024 575.376 1 0 1.50093 10 1.38182 1.4 0",
        'class': "muscle",
        'gainprm': "0.1628 1.1024 575.376 1 0 1.50093 10 1.38182 1.4 0",
        'lengthrange': "0.0575158 0.156738",
        'name': "addmagProx_r",
        'tendon': "addmagProx_r_tendon"
    },
    {
        'biasprm': "0.1 1.8 1328.94 1 0.05 3.5 10 1.98303 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.8 1328.94 1 0.05 3.5 10 1.98303 1.4 0",
        'lengthrange': "0.311093 0.518283",
        'name': "bflh_r",
        'tendon': "bflh_r_tendon"
    },
    {
        'biasprm': "0.406318 1.19782 586.382 1 0 1.59685 10 1.58092 1.4 0",
        'class': "muscle",
        'gainprm': "0.406318 1.19782 586.382 1 0 1.59685 10 1.58092 1.4 0",
        'lengthrange': "0.150634 0.237936",
        'name': "bfsh_r",
        'tendon': "bfsh_r_tendon"
    },
    {
        'biasprm': "0.620259 1.40474 553.241 1 0.319949 3.12081 10 1.35486 1.4 0",
        'class': "muscle",
        'gainprm': "0.620259 1.40474 553.241 1 0.319949 3.12081 10 1.35486 1.4 0",
        'lengthrange': "0.411859 0.466224",
        'name': "edl_r",
        'tendon': "edl_r_tendon"
    },
    {
        'biasprm': "0.696496 1.41065 265.537 1 0.344704 3.25853 10 1.48683 1.4 0",
        'class': "muscle",
        'gainprm': "0.696496 1.41065 265.537 1 0.344704 3.25853 10 1.48683 1.4 0",
        'lengthrange': "0.378893 0.432312",
        'name': "ehl_r",
        'tendon': "ehl_r_tendon"
    },
    {
        'biasprm': "0.598501 1.22918 332.13 1 0.357129 3.24643 10 1.8954 1.4 0",
        'class': "muscle",
        'gainprm': "0.598501 1.22918 332.13 1 0.357129 3.24643 10 1.8954 1.4 0",
        'lengthrange': "0.405466 0.433594",
        'name': "fdl_r",
        'tendon': "fdl_r_tendon"
    },
    {
        'biasprm': "0.562474 1.18745 749.779 1 0.292699 2.98554 10 1.03099 1.4 0",
        'class': "muscle",
        'gainprm': "0.562474 1.18745 749.779 1 0.292699 2.98554 10 1.03099 1.4 0",
        'lengthrange': "0.383982 0.416918",
        'name': "fhl_r",
        'tendon': "fhl_r_tendon"
    },
    {
        'biasprm': "0.1 1.49069 1318.66 1 0.05 3.5 10 1.90363 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.49069 1318.66 1 0.05 3.5 10 1.90363 1.4 0",
        'lengthrange': "0.366291 0.463723",
        'name': "gaslat_r",
        'tendon': "gaslat_r_tendon"
    },
    {
        'biasprm': "0.1 1.45 2237.29 1 0.0989568 3.5 10 1.51773 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.45 2237.29 1 0.0989568 3.5 10 1.51773 1.4 0",
        'lengthrange': "0.371968 0.472667",
        'name': "gasmed_r",
        'tendon': "gasmed_r_tendon"
    },
    {
        'biasprm': "0.680894 1.34594 1293.79 1 0 1.99579 10 1.86621 1.4 0",
        'class': "muscle",
        'gainprm': "0.680894 1.34594 1293.79 1 0 1.99579 10 1.86621 1.4 0",
        'lengthrange': "0.148932 0.246694",
        'name': "glmax1_r",
        'tendon': "glmax1_r_tendon"
    },
    {
        'biasprm': "0.684001 1.42041 1308.46 1 0 3.44601 10 1.22459 1.4 0",
        'class': "muscle",
        'gainprm': "0.684001 1.42041 1308.46 1 0 3.44601 10 1.22459 1.4 0",
        'lengthrange': "0.175275 0.290891",
        'name': "glmax2_r",
        'tendon': "glmax2_r_tendon"
    },
    {
        'biasprm': "1.36664 1.8 928.184 1 0.824721 3.5 10 1.37074 1.4 0",
        'class': "muscle",
        'gainprm': "1.36664 1.8 928.184 1 0.824721 3.5 10 1.37074 1.4 0",
        'lengthrange': "0.297946 0.474778",
        'name': "glmax3_r",
        'tendon': "glmax3_r_tendon"
    },
    {
        'biasprm': "0.103084 1.4555 1018.46 1 0 3.49286 10 1.55125 1.4 0",
        'class': "muscle",
        'gainprm': "0.103084 1.4555 1018.46 1 0 3.49286 10 1.55125 1.4 0",
        'lengthrange': "0.063342 0.162068",
        'name': "glmed1_r",
        'tendon': "glmed1_r_tendon"
    },
    {
        'biasprm': "0.147694 1.46079 715.019 1 0 3.5 10 1.62169 1.4 0",
        'class': "muscle",
        'gainprm': "0.147694 1.46079 715.019 1 0 3.5 10 1.62169 1.4 0",
        'lengthrange': "0.0760304 0.171887",
        'name': "glmed2_r",
        'tendon': "glmed2_r_tendon"
    },
    {
        'biasprm': "0.125877 1.51901 828.084 1 0 3.5 10 1.38543 1.4 0",
        'class': "muscle",
        'gainprm': "0.125877 1.51901 828.084 1 0 3.5 10 1.38543 1.4 0",
        'lengthrange': "0.0544219 0.156121",
        'name': "glmed3_r",
        'tendon': "glmed3_r_tendon"
    },
    {
        'biasprm': "0.1 1.44635 357.161 1 0.0576443 3.07057 10 1.2057 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.44635 357.161 1 0.0576443 3.07057 10 1.2057 1.4 0",
        'lengthrange': "0.0229136 0.11449",
        'name': "glmin1_r",
        'tendon': "glmin1_r_tendon"
    },
    {
        'biasprm': "0.1 1.60052 392.017 1 0.05 3.5 10 1.20071 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.60052 392.017 1 0.05 3.5 10 1.20071 1.4 0",
        'lengthrange': "0.0298796 0.115728",
        'name': "glmin2_r",
        'tendon': "glmin2_r_tendon"
    },
    {
        'biasprm': "0.1 1.8 386.063 1 0.0906571 3.5 10 1.40703 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.8 386.063 1 0.0906571 3.5 10 1.40703 1.4 0",
        'lengthrange': "0.0345943 0.11994",
        'name': "glmin3_r",
        'tendon': "glmin3_r_tendon"
    },
    {
        'biasprm': "0.652458 1.62414 280.917 1 0.00265608 3.5 10 0.938556 1.4 0",
        'class': "muscle",
        'gainprm': "0.652458 1.62414 280.917 1 0.00265608 3.5 10 0.938556 1.4 0",
        'lengthrange': "0.320644 0.541993",
        'name': "grac_r",
        'tendon': "grac_r_tendon"
    },
    {
        'biasprm': "0.24456 1.23695 977.408 1 0.0058227 1.83512 10 1.82107 1.4 0",
        'class': "muscle",
        'gainprm': "0.24456 1.23695 977.408 1 0.0058227 1.83512 10 1.82107 1.4 0",
        'lengthrange': "0.122191 0.22798",
        'name': "iliacus_r",
        'tendon': "iliacus_r_tendon"
    },
    {
        'biasprm': "0.845027 1.32175 496.045 1 0.461019 3.09903 10 1.80009 1.4 0",
        'class': "muscle",
        'gainprm': "0.845027 1.32175 496.045 1 0.461019 3.09903 10 1.80009 1.4 0",
        'lengthrange': "0.185892 0.207535",
        'name': "perbrev_r",
        'tendon': "perbrev_r_tendon"
    },
    {
        'biasprm': "0.816281 1.3032 993.42 1 0.530617 3.45831 10 1.14971 1.4 0",
        'class': "muscle",
        'gainprm': "0.816281 1.3032 993.42 1 0.530617 3.45831 10 1.14971 1.4 0",
        'lengthrange': "0.373688 0.398423",
        'name': "perlong_r",
        'tendon': "perlong_r_tendon"
    },
    {
        'biasprm': "0.1 1.8 819.319 1 0.1 3.5 10 1.62632 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.8 819.319 1 0.1 3.5 10 1.62632 1.4 0",
        'lengthrange': "0.0953594 0.167006",
        'name': "piri_r",
        'tendon': "piri_r_tendon"
    },
    {
        'biasprm': "0.497633 1.12953 1385.41 1 0.0134813 1.52019 10 1.47869 1.4 0",
        'class': "muscle",
        'gainprm': "0.497633 1.12953 1385.41 1 0.0134813 1.52019 10 1.47869 1.4 0",
        'lengthrange': "0.157716 0.231585",
        'name': "psoas_r",
        'tendon': "psoas_r_tendon"
    },
    {
        'biasprm': "0.1 1.8 2211.57 1 0.1 3.5 10 1.78166 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.8 2211.57 1 0.1 3.5 10 1.78166 1.4 0",
        'lengthrange': "0.423402 0.713662",
        'name': "recfem_r",
        'tendon': "recfem_r_tendon"
    },
    {
        'biasprm': "0.602155 1.16939 245.514 1 0 3.5 10 0.806615 1.4 0",
        'class': "muscle",
        'gainprm': "0.602155 1.16939 245.514 1 0 3.5 10 0.806615 1.4 0",
        'lengthrange': "0.366668 0.595265",
        'name': "sart_r",
        'tendon': "sart_r_tendon"
    },
    {
        'biasprm': "0.1 1.8 2249.55 1 0.0886069 3.5 10 1.07055 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.8 2249.55 1 0.0886069 3.5 10 1.07055 1.4 0",
        'lengthrange': "0.299644 0.504801",
        'name': "semimem_r",
        'tendon': "semimem_r_tendon"
    },
    {
        'biasprm': "0.389685 1.68623 583.121 1 0 3.5 10 1.57478 1.4 0",
        'class': "muscle",
        'gainprm': "0.389685 1.68623 583.121 1 0 3.5 10 1.57478 1.4 0",
        'lengthrange': "0.322408 0.572642",
        'name': "semiten_r",
        'tendon': "semiten_r_tendon"
    },
    {
        'biasprm': "0.360484 1.51929 5322.19 1 0.142043 3.39743 10 1.5467 1.4 0",
        'class': "muscle",
        'gainprm': "0.360484 1.51929 5322.19 1 0.142043 3.39743 10 1.5467 1.4 0",
        'lengthrange': "0.292617 0.343605",
        'name': "soleus_r",
        'tendon': "soleus_r_tendon"
    },
    {
        'biasprm': "0.1 1.46235 265.756 1 0.1 3.28179 10 1.33831 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.46235 265.756 1 0.1 3.28179 10 1.33831 1.4 0",
        'lengthrange': "0.373121 0.588421",
        'name': "tfl_r",
        'tendon': "tfl_r_tendon"
    },
    {
        'biasprm': "0.601012 1.38869 1152.86 1 0.241731 2.70077 10 1.34727 1.4 0",
        'class': "muscle",
        'gainprm': "0.601012 1.38869 1152.86 1 0.241731 2.70077 10 1.34727 1.4 0",
        'lengthrange': "0.28151 0.335308",
        'name': "tibant_r",
        'tendon': "tibant_r_tendon"
    },
    {
        'biasprm': "0.627194 1.23083 1434.86 1 0.363757 2.9639 10 1.34057 1.4 0",
        'class': "muscle",
        'gainprm': "0.627194 1.23083 1434.86 1 0.363757 2.9639 10 1.34057 1.4 0",
        'lengthrange': "0.304488 0.327305",
        'name': "tibpost_r",
        'tendon': "tibpost_r_tendon"
    },
    {
        'biasprm': "0.804197 1.8 1759.46 1 0.408078 3.5 10 0.993224 1.4 0",
        'class': "muscle",
        'gainprm': "0.804197 1.8 1759.46 1 0.408078 3.5 10 0.993224 1.4 0",
        'lengthrange': "0.282067 0.454352",
        'name': "vasint_r",
        'tendon': "vasint_r_tendon"
    },
    {
        'biasprm': "0.754438 1.8 5318.07 1 0.3408 3.5 10 2 1.4 0",
        'class': "muscle",
        'gainprm': "0.754438 1.8 5318.07 1 0.3408 3.5 10 2 1.4 0",
        'lengthrange': "0.295592 0.476057",
        'name': "vaslat_r",
        'tendon': "vaslat_r_tendon"
    },
    {
        'biasprm': "1.14989 1.61299 2423.69 1 0.682608 3.5 10 1.40617 1.4 0",
        'class': "muscle",
        'gainprm': "1.14989 1.61299 2423.69 1 0.682608 3.5 10 1.40617 1.4 0",
        'lengthrange': "0.311214 0.356041",
        'name': "vasmed_r",
        'tendon': "vasmed_r_tendon"
    },
    {
        'biasprm': "0.381903 1.46199 602.897 1 0.0683724 3.07879 10 1.3614 1.4 0",
        'class': "muscle",
        'gainprm': "0.381903 1.46199 602.897 1 0.0683724 3.07879 10 1.3614 1.4 0",
        'lengthrange': "0.0748245 0.186181",
        'name': "addbrev_l",
        'tendon': "addbrev_l_tendon"
    },
    {
        'biasprm': "0.361607 1.56839 881.343 1 0.125586 3.49915 10 1.54262 1.4 0",
        'class': "muscle",
        'gainprm': "0.361607 1.56839 881.343 1 0.125586 3.49915 10 1.54262 1.4 0",
        'lengthrange': "0.170925 0.301499",
        'name': "addlong_l",
        'tendon': "addlong_l_tendon"
    },
    {
        'biasprm': "0.647072 1.32551 581.265 1 0.0381507 2.10964 10 1.23765 1.4 0",
        'class': "muscle",
        'gainprm': "0.647072 1.32551 581.265 1 0.0381507 2.10964 10 1.23765 1.4 0",
        'lengthrange': "0.202042 0.322261",
        'name': "addmagDist_l",
        'tendon': "addmagDist_l_tendon"
    },
    {
        'biasprm': "0.662907 1.54717 566.285 1 0.139716 3.49861 10 1.14344 1.4 0",
        'class': "muscle",
        'gainprm': "0.662907 1.54717 566.285 1 0.139716 3.49861 10 1.14344 1.4 0",
        'lengthrange': "0.319889 0.458011",
        'name': "addmagIsch_l",
        'tendon': "addmagIsch_l_tendon"
    },
    {
        'biasprm': "0.477896 1.27927 578.677 1 0.0208952 2.02504 10 0.870476 1.4 0",
        'class': "muscle",
        'gainprm': "0.477896 1.27927 578.677 1 0.0208952 2.02504 10 0.870476 1.4 0",
        'lengthrange': "0.112434 0.222784",
        'name': "addmagMid_l",
        'tendon': "addmagMid_l_tendon"
    },
    {
        'biasprm': "0.1628 1.1024 568.695 1 0 2.16935 10 1.06805 1.4 0",
        'class': "muscle",
        'gainprm': "0.1628 1.1024 568.695 1 0 2.16935 10 1.06805 1.4 0",
        'lengthrange': "0.0575158 0.156738",
        'name': "addmagProx_l",
        'tendon': "addmagProx_l_tendon"
    },
    {
        'biasprm': "0.1 1.8 1327.75 1 0.05 3.5 10 1.75486 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.8 1327.75 1 0.05 3.5 10 1.75486 1.4 0",
        'lengthrange': "0.311093 0.518283",
        'name': "bflh_l",
        'tendon': "bflh_l_tendon"
    },
    {
        'biasprm': "0.406318 1.19782 556.777 1 0 3.14497 10 0.922935 1.4 0",
        'class': "muscle",
        'gainprm': "0.406318 1.19782 556.777 1 0 3.14497 10 0.922935 1.4 0",
        'lengthrange': "0.150634 0.237936",
        'name': "bfsh_l",
        'tendon': "bfsh_l_tendon"
    },
    {
        'biasprm': "0.620259 1.40474 545.598 1 0.313821 3.47851 10 1.79438 1.4 0",
        'class': "muscle",
        'gainprm': "0.620259 1.40474 545.598 1 0.313821 3.47851 10 1.79438 1.4 0",
        'lengthrange': "0.411859 0.466224",
        'name': "edl_l",
        'tendon': "edl_l_tendon"
    },
    {
        'biasprm': "0.696495 1.41066 266.688 1 0.343273 3.26938 10 1.3865 1.4 0",
        'class': "muscle",
        'gainprm': "0.696495 1.41066 266.688 1 0.343273 3.26938 10 1.3865 1.4 0",
        'lengthrange': "0.378893 0.432312",
        'name': "ehl_l",
        'tendon': "ehl_l_tendon"
    },
    {
        'biasprm': "0.5985 1.22918 331.887 1 0.352282 3.35018 10 1.69894 1.4 0",
        'class': "muscle",
        'gainprm': "0.5985 1.22918 331.887 1 0.352282 3.35018 10 1.69894 1.4 0",
        'lengthrange': "0.405466 0.433594",
        'name': "fdl_l",
        'tendon': "fdl_l_tendon"
    },
    {
        'biasprm': "0.562475 1.18745 746.517 1 0.288301 3.5 10 0.896858 1.4 0",
        'class': "muscle",
        'gainprm': "0.562475 1.18745 746.517 1 0.288301 3.5 10 0.896858 1.4 0",
        'lengthrange': "0.383982 0.416918",
        'name': "fhl_l",
        'tendon': "fhl_l_tendon"
    },
    {
        'biasprm': "0.1 1.49069 1324.37 1 0.05 3.5 10 1.93411 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.49069 1324.37 1 0.05 3.5 10 1.93411 1.4 0",
        'lengthrange': "0.366291 0.463723",
        'name': "gaslat_l",
        'tendon': "gaslat_l_tendon"
    },
    {
        'biasprm': "0.1 1.45 2245.47 1 0.0869909 3.46941 10 1.38125 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.45 2245.47 1 0.0869909 3.46941 10 1.38125 1.4 0",
        'lengthrange': "0.371968 0.472667",
        'name': "gasmed_l",
        'tendon': "gasmed_l_tendon"
    },
    {
        'biasprm': "0.680894 1.34594 1272.23 1 0 2.05861 10 1.422 1.4 0",
        'class': "muscle",
        'gainprm': "0.680894 1.34594 1272.23 1 0 2.05861 10 1.422 1.4 0",
        'lengthrange': "0.148932 0.246694",
        'name': "glmax1_l",
        'tendon': "glmax1_l_tendon"
    },
    {
        'biasprm': "0.684001 1.42041 1308.55 1 0.000794702 3.38516 10 0.806346 1.4 0",
        'class': "muscle",
        'gainprm': "0.684001 1.42041 1308.55 1 0.000794702 3.38516 10 0.806346 1.4 0",
        'lengthrange': "0.175275 0.290891",
        'name': "glmax2_l",
        'tendon': "glmax2_l_tendon"
    },
    {
        'biasprm': "1.36664 1.8 929.762 1 0.322928 3.5 10 1.93452 1.4 0",
        'class': "muscle",
        'gainprm': "1.36664 1.8 929.762 1 0.322928 3.5 10 1.93452 1.4 0",
        'lengthrange': "0.297946 0.474778",
        'name': "glmax3_l",
        'tendon': "glmax3_l_tendon"
    },
    {
        'biasprm': "0.103084 1.4555 1018.93 1 4.15271e-06 3.48299 10 1.46926 1.4 0",
        'class': "muscle",
        'gainprm': "0.103084 1.4555 1018.93 1 4.15271e-06 3.48299 10 1.46926 1.4 0",
        'lengthrange': "0.063342 0.162068",
        'name': "glmed1_l",
        'tendon': "glmed1_l_tendon"
    },
    {
        'biasprm': "0.147694 1.46079 716.963 1 0 3.39296 10 1.74572 1.4 0",
        'class': "muscle",
        'gainprm': "0.147694 1.46079 716.963 1 0 3.39296 10 1.74572 1.4 0",
        'lengthrange': "0.0760304 0.171887",
        'name': "glmed2_l",
        'tendon': "glmed2_l_tendon"
    },
    {
        'biasprm': "0.125877 1.51901 826.643 1 0.000637198 3.49115 10 1.65163 1.4 0",
        'class': "muscle",
        'gainprm': "0.125877 1.51901 826.643 1 0.000637198 3.49115 10 1.65163 1.4 0",
        'lengthrange': "0.0544219 0.156121",
        'name': "glmed3_l",
        'tendon': "glmed3_l_tendon"
    },
    {
        'biasprm': "0.1 1.44635 357.562 1 0.0602573 3.02127 10 1.3101 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.44635 357.562 1 0.0602573 3.02127 10 1.3101 1.4 0",
        'lengthrange': "0.0229136 0.11449",
        'name': "glmin1_l",
        'tendon': "glmin1_l_tendon"
    },
    {
        'biasprm': "0.1 1.60052 391.417 1 0.1 3.5 10 1.19315 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.60052 391.417 1 0.1 3.5 10 1.19315 1.4 0",
        'lengthrange': "0.0298796 0.115728",
        'name': "glmin2_l",
        'tendon': "glmin2_l_tendon"
    },
    {
        'biasprm': "0.1 1.8 389.612 1 0.0939983 3.5 10 1.15519 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.8 389.612 1 0.0939983 3.5 10 1.15519 1.4 0",
        'lengthrange': "0.0345943 0.11994",
        'name': "glmin3_l",
        'tendon': "glmin3_l_tendon"
    },
    {
        'biasprm': "0.652458 1.62414 280.263 1 0 3.5 10 1.68108 1.4 0",
        'class': "muscle",
        'gainprm': "0.652458 1.62414 280.263 1 0 3.5 10 1.68108 1.4 0",
        'lengthrange': "0.320644 0.541993",
        'name': "grac_l",
        'tendon': "grac_l_tendon"
    },
    {
        'biasprm': "0.24456 1.23695 974.831 1 0 1.8511 10 1.69465 1.4 0",
        'class': "muscle",
        'gainprm': "0.24456 1.23695 974.831 1 0 1.8511 10 1.69465 1.4 0",
        'lengthrange': "0.122191 0.22798",
        'name': "iliacus_l",
        'tendon': "iliacus_l_tendon"
    },
    {
        'biasprm': "0.845027 1.32175 496.73 1 0.467464 3.06759 10 1.61433 1.4 0",
        'class': "muscle",
        'gainprm': "0.845027 1.32175 496.73 1 0.467464 3.06759 10 1.61433 1.4 0",
        'lengthrange': "0.185892 0.207535",
        'name': "perbrev_l",
        'tendon': "perbrev_l_tendon"
    },
    {
        'biasprm': "0.816281 1.3032 997.426 1 0.540714 3.40374 10 1.41808 1.4 0",
        'class': "muscle",
        'gainprm': "0.816281 1.3032 997.426 1 0.540714 3.40374 10 1.41808 1.4 0",
        'lengthrange': "0.373688 0.398423",
        'name': "perlong_l",
        'tendon': "perlong_l_tendon"
    },
    {
        'biasprm': "0.1 1.8 822.575 1 0.0713008 3.5 10 0.992879 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.8 822.575 1 0.0713008 3.5 10 0.992879 1.4 0",
        'lengthrange': "0.0953594 0.167006",
        'name': "piri_l",
        'tendon': "piri_l_tendon"
    },
    {
        'biasprm': "0.497633 1.12953 1366.97 1 0.0118191 1.73378 10 1.03401 1.4 0",
        'class': "muscle",
        'gainprm': "0.497633 1.12953 1366.97 1 0.0118191 1.73378 10 1.03401 1.4 0",
        'lengthrange': "0.157716 0.231585",
        'name': "psoas_l",
        'tendon': "psoas_l_tendon"
    },
    {
        'biasprm': "0.1 1.8 2205.94 1 0.1 3.5 10 1.95607 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.8 2205.94 1 0.1 3.5 10 1.95607 1.4 0",
        'lengthrange': "0.423402 0.713662",
        'name': "recfem_l",
        'tendon': "recfem_l_tendon"
    },
    {
        'biasprm': "0.602155 1.16939 248.635 1 0.00170046 1.82656 10 1.79532 1.4 0",
        'class': "muscle",
        'gainprm': "0.602155 1.16939 248.635 1 0.00170046 1.82656 10 1.79532 1.4 0",
        'lengthrange': "0.366668 0.595265",
        'name': "sart_l",
        'tendon': "sart_l_tendon"
    },
    {
        'biasprm': "0.1 1.8 2196.22 1 0.05 3.49955 10 1.30483 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.8 2196.22 1 0.05 3.49955 10 1.30483 1.4 0",
        'lengthrange': "0.299644 0.504801",
        'name': "semimem_l",
        'tendon': "semimem_l_tendon"
    },
    {
        'biasprm': "0.389685 1.68623 579.776 1 0.0312812 3.5 10 1.94751 1.4 0",
        'class': "muscle",
        'gainprm': "0.389685 1.68623 579.776 1 0.0312812 3.5 10 1.94751 1.4 0",
        'lengthrange': "0.322408 0.572642",
        'name': "semiten_l",
        'tendon': "semiten_l_tendon"
    },
    {
        'biasprm': "0.360484 1.51929 5284.56 1 0.130611 3.5 10 1.35441 1.4 0",
        'class': "muscle",
        'gainprm': "0.360484 1.51929 5284.56 1 0.130611 3.5 10 1.35441 1.4 0",
        'lengthrange': "0.292617 0.343605",
        'name': "soleus_l",
        'tendon': "soleus_l_tendon"
    },
    {
        'biasprm': "0.1 1.46235 262.053 1 0.1 3.49308 10 1.13577 1.4 0",
        'class': "muscle",
        'gainprm': "0.1 1.46235 262.053 1 0.1 3.49308 10 1.13577 1.4 0",
        'lengthrange': "0.373121 0.588421",
        'name': "tfl_l",
        'tendon': "tfl_l_tendon"
    },
    {
        'biasprm': "0.601012 1.38869 1143.45 1 0.235755 3.05219 10 1.36022 1.4 0",
        'class': "muscle",
        'gainprm': "0.601012 1.38869 1143.45 1 0.235755 3.05219 10 1.36022 1.4 0",
        'lengthrange': "0.28151 0.335308",
        'name': "tibant_l",
        'tendon': "tibant_l_tendon"
    },
    {
        'biasprm': "0.627194 1.23083 1429.97 1 0.35657 3.14947 10 1.76838 1.4 0",
        'class': "muscle",
        'gainprm': "0.627194 1.23083 1429.97 1 0.35657 3.14947 10 1.76838 1.4 0",
        'lengthrange': "0.304488 0.327305",
        'name': "tibpost_l",
        'tendon': "tibpost_l_tendon"
    },
    {
        'biasprm': "0.804197 1.8 1759.59 1 0.403463 3.5 10 1.60134 1.4 0",
        'class': "muscle",
        'gainprm': "0.804197 1.8 1759.59 1 0.403463 3.5 10 1.60134 1.4 0",
        'lengthrange': "0.282067 0.454352",
        'name': "vasint_l",
        'tendon': "vasint_l_tendon"
    },
    {
        'biasprm': "0.754438 1.8 5322.59 1 0.340667 3.5 10 1.80773 1.4 0",
        'class': "muscle",
        'gainprm': "0.754438 1.8 5322.59 1 0.340667 3.5 10 1.80773 1.4 0",
        'lengthrange': "0.295592 0.476057",
        'name': "vaslat_l",
        'tendon': "vaslat_l_tendon"
    },
    {
        'biasprm': "1.14989 1.61299 2422.17 1 0.412194 3.49301 10 1.4918 1.4 0",
        'class': "muscle",
        'gainprm': "1.14989 1.61299 2422.17 1 0.412194 3.49301 10 1.4918 1.4 0",
        'lengthrange': "0.311214 0.356041",
        'name': "vasmed_l",
        'tendon': "vasmed_l_tendon"
    }
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

#Add equality
equality_node = ET.SubElement(root, 'equality')
joints = [
{'joint1': "knee_angle_r_translation2", 'joint2': "knee_angle_r", 'name': "knee_angle_r_translation2_constraint", 'polycoef': "7.69254e-11 0.00587971 -0.00125622 -2.61846e-06 6.24355e-07", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_r_translation1", 'joint2': "knee_angle_r", 'name': "knee_angle_r_translation1_constraint", 'polycoef': "9.53733e-08 0.00312879 -0.00230804 0.000561561 5.68366e-07", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_r_rotation2", 'joint2': "knee_angle_r", 'name': "knee_angle_r_rotation2_constraint", 'polycoef': "-1.47325e-08 0.0791 -0.0328478 -0.0252183 0.0108321", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_r_rotation3", 'joint2': "knee_angle_r", 'name': "knee_angle_r_rotation3_constraint", 'polycoef': "1.08939e-08 0.369499 -0.169478 0.0251643 3.50498e-07", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_r_beta_translation2", 'joint2': "knee_angle_r", 'name': "knee_angle_r_beta_translation2_constraint", 'polycoef': "-0.0108281 -0.0487847 0.00927644 0.0131673 -0.00349673", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_r_beta_translation1", 'joint2': "knee_angle_r", 'name': "knee_angle_r_beta_translation1_constraint", 'polycoef': "0.0524192 -0.0150188 -0.0340522 0.0133393 -0.000879151", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_r_beta_rotation1", 'joint2': "knee_angle_r", 'name': "knee_angle_r_beta_rotation1_constraint", 'polycoef': "0.010506 0.0247615 -1.31647 0.716337 -0.138302", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_l_translation2", 'joint2': "knee_angle_l", 'name': "knee_angle_l_translation2_constraint", 'polycoef': "-7.69254e-11 -0.00587971 0.00125622 2.61846e-06 -6.24355e-07", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_l_translation1", 'joint2': "knee_angle_l", 'name': "knee_angle_l_translation1_constraint", 'polycoef': "9.53733e-08 0.00312879 -0.00230804 0.000561561 5.68366e-07", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_l_rotation2", 'joint2': "knee_angle_l", 'name': "knee_angle_l_rotation2_constraint", 'polycoef': "-1.47325e-08 0.0791 -0.0328478 -0.0252183 0.0108321", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_l_rotation3", 'joint2': "knee_angle_l", 'name': "knee_angle_l_rotation3_constraint", 'polycoef': "-1.08939e-08 -0.369499 0.169478 -0.0251643 -3.50498e-07", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_l_beta_translation2", 'joint2': "knee_angle_l", 'name': "knee_angle_l_beta_translation2_constraint", 'polycoef': "-0.0108281 -0.0487847 0.00927644 0.0131673 -0.00349673", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_l_beta_translation1", 'joint2': "knee_angle_l", 'name': "knee_angle_l_beta_translation1_constraint", 'polycoef': "0.0524192 -0.0150188 -0.0340522 0.0133393 -0.000879151", 'solimp': "0.9999 0.9999 0.001 0.5 2"},
{'joint1': "knee_angle_l_beta_rotation1", 'joint2': "knee_angle_l", 'name': "knee_angle_l_beta_rotation1_constraint", 'polycoef': "0.010506 0.0247615 -1.31647 0.716337 -0.138302", 'solimp': "0.9999 0.9999 0.001 0.5 2"}
]

for joint in joints:
    joint_node = ET.SubElement(equality_node, 'joint')
    joint_node.set('joint1', joint['joint1'])
    joint_node.set('joint2', joint['joint2'])
    joint_node.set('name', joint['name'])
    joint_node.set('polycoef', joint['polycoef'])
    joint_node.set('solimp', joint['solimp'])

# Sensors
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

xml_str = ET.tostring(root, encoding="unicode", method="xml")
xml_str = xml.dom.minidom.parseString(xml_str).toprettyxml(indent="  ", newl="\n")

with open("generate_mujoco_model.xml", "w") as f:
    f.write(xml_str)
