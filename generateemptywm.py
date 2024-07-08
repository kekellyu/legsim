import xml.etree.ElementTree as ET
import xml.dom.minidom

def parse_xml(file_path):
    tree = ET.parse(file_path)
    return tree, tree.getroot()

def get_position(root, body_name):
    body = root.find(f".//body[@name='{body_name}']")
    if body is not None:
        pos_str = body.get('pos')
        return list(map(float, pos_str.split()))
    else:
        return None

def get_mass(root, body_name):
    body = root.find(f".//body[@name='{body_name}']")
    if body is not None:
        inertial = body.find("inertial")
        if inertial is not None:
            mass = inertial.get('mass')
            return float(mass) if mass is not None else None
    return None

#Get Data from xml

file_path='emptywmuscle.xml'
tree, root = parse_xml(file_path)

ExoLeftSagittalHipLink_pos = get_position(root, 'LeftSagittalHipLink')
ExoLeftSagittalKneeLink_pos = get_position(root, 'LeftSagittalKneeLink')
ExoLeftSagittalAnkleLink_pos = get_position(root,'LeftSagittalAnkleLink')
ExoLeftFrontalHip_pos= get_position(root, 'LeftFrontalHipLink')
ExoLeftTransverseHipLink_pos= get_position(root, 'LeftTransverseHipLink')

ExoRightSagittalHipLink_pos = get_position(root, 'RightSagittalHipLink')
ExoRightSagittalKneeLink_pos = get_position(root, 'RightSagittalKneeLink')
ExoRightSagittalAnkleLink_pos = get_position(root,'RightSagittalAnkleLink')
ExoRightFrontalHip_pos= get_position(root, 'RightFrontalHipLink')
ExoRightTransverseHipLink_pos= get_position(root, 'RightTransverseHipLink')

Exotorso_mass= get_mass(root, 'torso')
ExoLeftFrontalHip_mass= get_mass(root, 'LeftFrontalHipLink')
ExoLeftTransverseHipLink_mass= get_mass(root, 'LeftTransverseHipLink')
ExoLeftSagittalHipLink_mass= get_mass(root, 'LeftSagittalHipLink')
ExoLeftSagittalKneeLink_mass= get_mass(root, 'LeftSagittalKneeLink')
ExoLeftSagittalAnkleLink_mass= get_mass(root, 'LeftSagittalAnkleLink')
ExoLeftHenkeAnkleLink_mass= get_mass(root, 'LeftHenkeAnkleLink')
ExoRightFrontalHip_mass= get_mass(root, 'RightFrontalHipLink')
ExoRightTransverseHipLink_mass= get_mass(root, 'RightTransverseHipLink')
ExoRightSagittalHipLink_mass= get_mass(root, 'RightSagittalHipLink')
ExoRightSagittalKneeLink_mass= get_mass(root, 'RightSagittalKneeLink')
ExoRightSagittalAnkleLink_mass= get_mass(root, 'RightSagittalAnkleLink')
ExoRightHenkeAnkleLink_mass= get_mass(root, 'RightHenkeAnkleLink')

#Change name

a, b, c = ExoLeftSagittalHipLink_pos
d, e, f = ExoLeftSagittalKneeLink_pos
g, h, i = ExoLeftSagittalAnkleLink_pos
aa, bb, cc = ExoLeftFrontalHip_pos
aaa, bbb, ccc = ExoLeftTransverseHipLink_pos

j, k, l = ExoRightSagittalHipLink_pos
m, n, o = ExoRightSagittalKneeLink_pos
p, q, r = ExoRightSagittalAnkleLink_pos
jj, kk, ll = ExoRightFrontalHip_pos
jjj, kkk, lll = ExoRightTransverseHipLink_pos

m1 = Exotorso_mass
m2 = ExoLeftFrontalHip_mass
m3 = ExoLeftTransverseHipLink_mass
m4 = ExoLeftSagittalHipLink_mass
m5 = ExoLeftSagittalKneeLink_mass
m6 = ExoLeftSagittalAnkleLink_mass
m7 = ExoLeftHenkeAnkleLink_mass
m8 = ExoRightFrontalHip_mass
m9 = ExoRightTransverseHipLink_mass
m10 = ExoRightSagittalHipLink_mass
m11 = ExoRightSagittalKneeLink_mass
m12 = ExoRightSagittalAnkleLink_mass
m13 = ExoRightHenkeAnkleLink_mass

#Positions

#Exoskeleton Link Positions
Exotorso_pos=[0,0,1]
# ExoLeftFrontalHip_pos=[0,0.089,0]
# ExoLeftTransverseHipLink_pos=[-0.135,0.169,0]
# ExoLeftSagittalHipLink_pos=[0.135,0,0]
# ExoLeftSagittalKneeLink_pos=[0,0.0049114,-0.38]
# ExoLeftSagittalAnkleLink_pos=[0,-0.16942,-0.408]
# ExoRightFrontalHip_pos=[0,-0.089,0]
# ExoRightTransverseHipLink_pos=[-0.135,-0.169,0]
# ExoRightSagittalHipLink_pos=[0.135,0,0]
# ExoRightSagittalKneeLink_pos=[0,-0.0049114,-0.38]
# ExoRightSagittalAnkleLink_pos=[0,0.16942,-0.408]

#Muscle Positions
# Y is the height
Muscletorso_pos=[0,0,1]

Musclefemur_r_pos=[-0.056276*a/0.135,-0.07849*(b+1),0.07726*(c+1)]
Muscletibia_r_pos=[-4.6e-07*(d+1),-0.404425*e/0.0049114,-0.00126526*f/(-0.38)]
Muscletalus_r_pos=[-0.01*(g+1),-0.4*h/(-0.16942),0]

Musclecalcn_r_pos=[-0.04877,-0.04195,0.00792]
Muscletoes_r_pos=[0.1788,-0.002,0.00108]
Musclepatella_r_pos=[-0.00809,-0.40796,0]

Musclefemur_l_pos=[-0.056276*j/0.135,-0.07849*(k+1),-0.07726*(l+1)]
Muscletibia_l_pos=[-4.6e-07*(m+1),-0.404425*n/(-0.0049114),0.00126526*o/(-0.38)]
Muscletalus_l_pos=[-0.01*(p+1),-0.4*q/0.16942,0]

Musclecalcn_l_pos=[-0.04877,-0.04195,-0.00792]
Muscletoes_l_pos=[0.1788,-0.002,-0.00108]
Musclepatella_l_pos=[-0.00809,-0.40796,0]

#Masses

#Exo Mass
# Exotorso_mass=[16.4638]
# ExoLeftFrontalHip_mass=[3.6752]
# ExoLeftTransverseHipLink_mass=[4.4176]
# ExoLeftSagittalHipLink_mass=[8.9468]
# ExoLeftSagittalKneeLink_mass=[10.9753]
# ExoLeftSagittalAnkleLink_mass=[1.6764]
# ExoLeftHenkeAnkleLink_mass=[3.2239]
# ExoRightFrontalHip_mass=[3.6752]
# ExoRightTransverseHipLink_mass=[4.4176]
# ExoRightSagittalHipLink_mass=[8.9468]
# ExoRightSagittalKneeLink_mass=[10.9753]
# ExoRightSagittalAnkleLink_mass=[1.6764]
# ExoRightHenkeAnkleLink_mass=[3.2239]

Exotorso_mass=[m1]
ExoLeftFrontalHip_mass=[m2]
ExoLeftTransverseHipLink_mass=[m3]
ExoLeftSagittalHipLink_mass=[m4]
ExoLeftSagittalKneeLink_mass=[m5]
ExoLeftSagittalAnkleLink_mass=[m6]
ExoLeftHenkeAnkleLink_mass=[m7]
ExoRightFrontalHip_mass=[m8]
ExoRightTransverseHipLink_mass=[m9]
ExoRightSagittalHipLink_mass=[m10]
ExoRightSagittalKneeLink_mass=[m11]
ExoRightSagittalAnkleLink_mass=[m12]
ExoRightHenkeAnkleLink_mass=[m13]

#Muscle Mass
Muscletorso_mass=[10.96*m1/16.4638]
Musclefemur_r_mass=[8.4*(m2/3.6752+m3/4.4176+m4/8.9468)]
Muscletibia_r_mass=[3.8*m5/10.9753]
Musclecalcn_r_mass=[1.14*(m6/1.6764+m7/3.2239)]
Musclefemur_l_mass=[8.4*(m8/3.6752+m9/4.4176+m10/8.9468)]
Muscletibia_l_mass=[3.8*m11/10.9753]
Musclecalcn_l_mass=[1.14*(m12/1.6764+m13/3.2239)]

# tree = ET.parse('emptywmuscle.xml')
root = ET.Element("mujoco")
root.set('model','Amy_Li_emptyExo')

compiler_node = ET.SubElement(root,'compiler')
compiler_node.set('angle', 'radian')
compiler_node.set('autolimits', 'true')

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

#Worldbody
# emptywmuscle_path = 'emptywmuscle.xml'
# tree = ET.parse(emptywmuscle_path)
# emptywmuscle_root = tree.getroot()
# worldbody_element = emptywmuscle_root.find('worldbody')
# worldbody_new = ET.SubElement(root,'worldbody')  
# worldbody_new.append(worldbody_element) 
#  # Add torso
# torso_node = ET.SubElement(worldbody_new, 'body')
# torso_node.set('name', 'torso')
# torso_node.set('pos', '0 0 1')
# # Add plane geom
# geom_plane = ET.SubElement(torso_node, 'geom')
# geom_plane.set('name', 'plane')
# geom_plane.set('type', 'plane')
# geom_plane.set('material', 'plane')
# geom_plane.set('size', '10 5 0.1')
# geom_plane.set('rgba', '0.9 0.9 0.9 1')
# geom_plane.set('condim', '3')
# geom_plane.set('conaffinity', '15')
# geom_plane.set('friction', '1')

# sites = [
#     {'name': "pelvissite", 'pos': "0 0 0"},
#     {'name': "LeftHipBack", 'pos': "-0.08 0.08 -0.04"},
#     {'name': "LeftHipFront", 'pos': "0.09 0.08 -0.04"},
#     {'name': "RightHipBack", 'pos': "-0.08 -0.08 -0.04"},
#     {'name': "RightHipFront", 'pos': "0.09 -0.08 -0.04"}
# ]

# for site in sites:
#     site_node = ET.SubElement(torso_node, 'site')
#     site_node.set('name', site['name'])
#     site_node.set('pos', site['pos'])
# import xml.etree.ElementTree as ET

worldbody_node = ET.SubElement(root, 'worldbody')

# Add the 'geom' element
terrain_geom = {
    'conaffinity': "3",
    'condim': "3",
    'friction': "1 .1 .1",
    'material': "plane",
    'name': "terrain_mesh",
    'rgba': ".9 .9 .9 1",
    'size': "10 5 0.1",
    'type': "plane"
}

geom_node = ET.SubElement(worldbody_node, 'geom')
for key, value in terrain_geom.items():
    geom_node.set(key, value)

# Function to add nested bodies recursively
def add_body(parent_node, body_data):
    body_node = ET.SubElement(parent_node, 'body', {'name': body_data['name'], 'pos': body_data['pos']})
    
    for site in body_data.get('sites', []):
        ET.SubElement(body_node, 'site', site)
    
    for inertial in body_data.get('inertials', []):
        ET.SubElement(body_node, 'inertial', inertial)
    
    for geom in body_data.get('geoms', []):
        ET.SubElement(body_node, 'geom', geom)
    
    for joint in body_data.get('joints', []):
        ET.SubElement(body_node, 'joint', joint)
    
    for child_body in body_data.get('bodies', []):
        add_body(body_node, child_body)

# Define the nested body structure
bodies_data = [
    {
        'name': "torso",
        'pos': " ".join(map(str,Exotorso_pos)),
        'sites': [
            {'name': "pelvissite", 'pos': "0 0 0"},
            {'name': "LeftHipBack", 'pos': "-0.08 0.08 -0.04"},
            {'name': "LeftHipFront", 'pos': "0.09 0.08 -0.04"},
            {'name': "RightHipBack", 'pos': "-0.08 -0.08 -0.04"},
            {'name': "RightHipFront", 'pos': "0.09 -0.08 -0.04"},
            {'group': "3", 'name': "pelvis_imu", 'pos': "-0.16445 -0.014625 0.19109", 'quat': "0.5 -0.5 -0.5 0.5", 'size': ".01"},
            {'group': "3", 'name': "thorax_imu", 'pos': "-0.14 0 0.5", 'quat': "0.70711 0 0.70711 0", 'size': ".01"}
        ],
        'inertials': [
            {'diaginertia': "0.6633 0.55313 0.18091", 'mass': " ".join(map(str,Exotorso_mass)), 'pos': "-0.18186 -0.00011 0.13746"}
        ],
        'geoms': [
            {'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "PelvisLink", 'type': "mesh"}
        ],
        'bodies': [
                {
                    'name': "LeftFrontalHipLink",
                    'pos': " ".join(map(str,ExoLeftFrontalHip_pos)),
                    'sites': [{'name': "LeftFrontalHipSite", 'pos': "0 0 0"}],
                    'inertials': [{'diaginertia': "0.036612 0.0349177 0.00740632", 'mass': " ".join(map(str,ExoLeftFrontalHip_mass)), 'pos': "-0.18696 0.095567 0.018365", 'quat': "0.453895 0.291805 -0.721185 0.43442"}],
                    'joints': [{'axis': "1 0 0", 'name': "LeftFrontalHipJoint", 'pos': "0 0 0", 'range': "-0.2 0.3"}],
                    'geoms': [{'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "LeftFrontalHipLink", 'type': "mesh"}],
                    'bodies': [
                        {
                            'name': "LeftTransverseHipLink",
                            'pos': " ".join(map(str,ExoLeftTransverseHipLink_pos)),
                            'sites': [{'name': "LeftTransverseHipSite", 'pos': "0 0 0"}],
                            'inertials': [{'diaginertia': "0.0259937 0.0259386 0.00622165", 'mass': " ".join(map(str,ExoLeftTransverseHipLink_mass)), 'pos': "0.069171 -0.008671 0.004224", 'quat': "-0.114265 0.696606 -0.205723 0.677762"}],
                            'joints': [{'axis': "0 0 1", 'name': "LeftTransverseHipJoint", 'pos': "0 0 0", 'range': "-0.2 0.3"}],
                            'geoms': [{'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "LeftTransverseHipLink", 'type': "mesh"}],
                            'bodies': [
                                {
                                    'name': "LeftSagittalHipLink",
                                    'pos': " ".join(map(str,ExoLeftSagittalHipLink_pos)),
                                    'sites': [{'name': "LeftSagittalHipSite", 'pos': "0 0 0"}],
                                    'inertials': [{'diaginertia': "0.17014 0.155437 0.0522649", 'mass': " ".join(map(str,ExoLeftSagittalHipLink_mass)), 'pos': "0.022786 -0.018046 -0.18739", 'quat': "0.518047 -0.0483842 0.0468759 0.852695"}],
                                    'joints': [{'axis': "0 1 0", 'name': "LeftSagittalHipJoint", 'pos': "0 0 0", 'range': "-2 0.3"}],
                                    'geoms': [{'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "LeftSagittalHipLink", 'type': "mesh"}],
                                    'bodies': [
                                        {
                                            'name': "LeftSagittalKneeLink",
                                            'pos': " ".join(map(str,ExoLeftSagittalKneeLink_pos)),
                                            'inertials': [{'diaginertia': "0.292576 0.283039 0.0639544", 'mass': " ".join(map(str,ExoLeftSagittalKneeLink_mass)),'pos': "-0.063222 -0.1107 -0.20848", 'quat': "0.96457 -0.15051 0.183544 -0.115166"}],
                                            'joints': [{'axis': "0 1 0", 'name': "LeftSagittalKneeJoint", 'pos': "0 0 0", 'range': "0 1.9"}],
                                            'geoms': [{'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "LeftSagittalKneeLink", 'type': "mesh"}],
                                            'sites': [
                                                {'group': "3", 'name': "left_tibia_imu", 'pos': "-0.14067 -0.074092 -0.39348", 'quat': "2.3108e-07 -2.3108e-07 0.70711 -0.70711", 'size': ".01"},
                                                {'name': "LeftSagittalKneeFront", 'pos': "0.06 0.013 0"},
                                                {'name': "LeftSagittalKneeBack", 'pos': "-0.06 0.013 0"},
                                                {'name': "LeftKneeFront", 'pos': "0.015 -0.16 -0.03"},
                                                {'name': "LeftKneeBack", 'pos': "-0.07 -0.16 -0.03"},
                                                {'name': "LeftSagittalKneeSite", 'pos': "0 0 0"}
                                            ],
                                            'bodies': [
                                                {
                                                    'name': "LeftSagittalAnkleLink",
                                                    'pos': " ".join(map(str,ExoLeftSagittalAnkleLink_pos)),
                                                    'quat': "0.997564 0 0 0.0697583",
                                                    'sites': [{'name': "LeftSagittalAnkleSite", 'pos': "0 0 0"}],
                                                    'inertials': [{'diaginertia': "0.00498533 0.00498533 0.00498533", 'mass': " ".join(map(str,ExoLeftSagittalAnkleLink_mass)), 'pos': "-0.043917 0.056353 -0.038953", 'quat': "0.390876 0.856905 0.0797629 -0.326448"}],
                                                    'joints': [{'axis': "0 1 0", 'name': "LeftSagittalAnkleJoint", 'pos': "0 0 0", 'range': "-0.3 0.2"}],
                                                    'geoms': [{'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "LeftSagittalAnkleLink", 'type': "mesh"}],
                                                    'bodies': [
                                                        {
                                                            'name': "LeftHenkeAnkleLink",
                                                            'pos': "0 0 0",
                                                            'inertials': [{'diaginertia': "0.0289377 0.0289377 0.0289377", 'mass': " ".join(map(str,ExoLeftHenkeAnkleLink_mass)), 'pos': "-0.053246 2e-06 -0.11063", 'quat': "0.280854 0.443077 0.216278 0.823424"}],
                                                            'joints': [{'axis': "0.788011 0 0.615661", 'name': "LeftHenkeAnkleJoint", 'pos': "0 0 0", 'range': "-0.3 0.3"}],
                                                            'geoms': [
                                                                {'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "LeftHenkeAnkleLink", 'quat': "0.945518 0 -0.32557 0", 'type': "mesh"},
                                                                {'condim': "4", 'friction': "1", 'name': "left_sole", 'pos': "0.06 -3.3e-05 -0.15975", 'quat': "0.707105 0 -0.707108 0", 'rgba': "1 0 0 1", 'size': "0.0049535 0.059645 0.1375", 'type': "box"},
                                                                {'condim': "4", 'friction': "1", 'name': "left_toe", 'pos': "0.2195 0 -0.1537", 'quat': "0.6018 0 -0.7986 0", 'rgba': "1 0 0 1", 'size': "0.0049535 0.059645 0.022", 'type': "box"},
                                                                {'condim': "4", 'friction': "1", 'name': "left_heel", 'pos': "-0.0940 0 -0.1566", 'quat': "0.7716 0 -0.6361 0", 'rgba': "1 0 0 1", 'size': "0.0049535 0.059645 0.0165", 'type': "box"}
                                                            ],
                                                            'sites': [
                                                                {'name': "LeftHenkeAnkleSite", 'pos': "0 0 0"},
                                                                {'name': "LeftHenkeAnkleFront", 'pos': "0.033 0.1 0"},
                                                                {'name': "LeftHenkeAnkleBack", 'pos': "-0.033 0.1 0"},
                                                                {'name': "LeftAnkleFront", 'pos': "0.025 0 -0.1"},
                                                                {'name': "LeftAnkleBack", 'pos': "-0.04 0 -0.1"},
                                                                {'name': "left_foot_imu", 'pos': "-0.036966 0.015 -0.13705", 'quat': "-1.6377e-07 -0.94552 -5.639e-08 0.32557", 'size': ".01"},
                                                                {'group': "2", 'name': "opto1", 'pos': "0.17825 0.0285 -0.1598", 'quat': "1 0 0 1", 'size': "0.05"},
                                                                {'group': "2", 'name': "opto2", 'pos': "0.17825 -0.0285 -0.1598", 'quat': "1 0 0 1", 'size': "0.05"},
                                                                {'group': "2", 'name': "opto3", 'pos': "-0.063753 0.02625 -0.1598", 'quat': "1 0 0 1", 'size': "0.05"},
                                                                {'group': "2", 'name': "opto4", 'pos': "-0.063753 -0.02625 -0.1598", 'quat': "1 0 0 1", 'size': "0.05"}
                                                            ]
                                                        }
                                                    ]
                                                }
                                            ]
                                        }
                                    ]
                                }
                            ]
                        }
                    ]
                },
            {
                'name': "RightFrontalHipLink",
                'pos': " ".join(map(str,ExoRightFrontalHip_pos)),
                'sites': [{'name': "RightFrontalHipSite", 'pos': "0 0 0"}],
                'inertials': [{'diaginertia': "0.0365973 0.0349128 0.00740791", 'mass': " ".join(map(str,ExoRightFrontalHip_mass)), 'pos': "-0.18696 -0.095567 0.01843", 'quat': "0.292578 0.454629 -0.434128 0.720585"}],
                'joints': [{'axis': "1 0 0", 'name': "RightFrontalHipJoint", 'pos': "0 0 0", 'range': "-0.3 0.2"}],
                'geoms': [{'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "RightFrontalHipLink", 'type': "mesh"}],
                'bodies': [
                    {
                        'name': "RightTransverseHipLink",
                        'pos': " ".join(map(str,ExoRightTransverseHipLink_pos)),
                        'sites': [{'name': "RightTransverseHipSite", 'pos': "0 0 0"}],
                        'inertials': [{'diaginertia': "0.0259901 0.0259386 0.00622121", 'mass': " ".join(map(str,ExoRightTransverseHipLink_mass)), 'pos': "0.069166 0.008663 0.004212", 'quat': "0.111328 0.697511 0.202883 0.678177"}],
                        'joints': [{'axis': "0 0 1", 'name': "RightTransverseHipJoint", 'pos': "0 0 0", 'range': "-0.3 0.2"}],
                        'geoms': [{'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "RightTransverseHipLink", 'type': "mesh"}],
                        'bodies': [
                            {
                                'name': "RightSagittalHipLink",
                                'pos': " ".join(map(str,ExoRightSagittalHipLink_pos)),
                                'sites': [{'name': "RightSagittalHipSite", 'pos': "0 0 0"}],
                                'inertials': [{'diaginertia': "0.17014 0.155437 0.0522649", 'mass': " ".join(map(str,ExoRightSagittalHipLink_mass)), 'pos': "0.022786 0.018046 -0.18739", 'quat': "0.852695 0.0468759 -0.0483842 0.518047"}],
                                'joints': [{'axis': "0 1 0", 'name': "RightSagittalHipJoint", 'pos': "0 0 0", 'range': "-2 0.3"}],
                                'geoms': [{'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "RightSagittalHipLink", 'type': "mesh"}],
                                'bodies': [
                                    {
                                        'name': "RightSagittalKneeLink",
                                        'pos': " ".join(map(str,ExoRightSagittalKneeLink_pos)),
                                        'inertials': [{'diaginertia': "0.292576 0.283039 0.0639544", 'mass': " ".join(map(str,ExoRightSagittalKneeLink_mass)), 'pos': "-0.063222 0.1107 -0.20848", 'quat': "0.96457 0.15051 0.183544 0.115166"}],
                                        'joints': [{'axis': "0 1 0", 'name': "RightSagittalKneeJoint", 'pos': "0 0 0", 'range': "0 1.9"}],
                                        'geoms': [{'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "RightSagittalKneeLink", 'type': "mesh"}],
                                        'sites': [
                                            {'group': "3", 'name': "right_tibia_imu", 'pos': "-0.11573 0.074342 -0.39348", 'quat': "0.70711 -0.70711 0 0", 'size': ".01"},
                                            {'name': "RightSagittalKneeFront", 'pos': "0.06 0.013 0"},
                                            {'name': "RightSagittalKneeBack", 'pos': "-0.06 0.013 0"},
                                            {'name': "RightKneeFront", 'pos': "0.015 0.16 -0.03"},
                                            {'name': "RightKneeBack", 'pos': "-0.07 0.16 -0.03"},
                                            {'name': "RightSagittalKneeSite", 'pos': "0 0 0"}
                                        ],
                                        'bodies': [
                                            {
                                                'name': "RightSagittalAnkleLink",
                                                'pos': " ".join(map(str,ExoRightSagittalAnkleLink_pos)),
                                                'quat': "0.997564 0 0 -0.0697583",
                                                'sites': [{'name': "RightSagittalAnkleSite", 'pos': "0 0 0"}],
                                                'inertials': [{'diaginertia': "0.00498533 0.00498533 0.00498533", 'mass': " ".join(map(str,ExoRightSagittalAnkleLink_mass)), 'pos': "-0.043917 -0.056352 -0.038953", 'quat': "0.856905 0.390876 0.326448 -0.0797629"}],
                                                'joints': [{'axis': "0 1 0", 'name': "RightSagittalAnkleJoint", 'pos': "0 0 0", 'range': "-0.3 0.2"}],
                                                'geoms': [{'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "RightSagittalAnkleLink", 'type': "mesh"}],
                                                'bodies': [
                                                    {
                                                        'name': "RightHenkeAnkleLink",
                                                        'pos': "0 0 0",
                                                        'inertials': [{'diaginertia': "0.0289377 0.0289377 0.0289377", 'mass': " ".join(map(str,ExoRightHenkeAnkleLink_mass)), 'pos': "-0.053246 -2e-06 -0.11063", 'quat': "-0.280854 0.443077 -0.216278 0.823424"}],
                                                        'joints': [{'axis': "0.788011 0 0.615661", 'name': "RightHenkeAnkleJoint", 'pos': "0 0 0", 'range': "-0.3 0.3"}],
                                                        'geoms': [
                                                            {'conaffinity': "0", 'contype': "0", 'density': "0", 'group': "1", 'mesh': "RightHenkeAnkleLink", 'quat': "0.945518 0 -0.32557 0", 'type': "mesh"},
                                                            {'condim': "4", 'friction': "1", 'name': "right_sole", 'pos': "0.06 -3.3e-05 -0.15975", 'quat': "0.707105 0 -0.707108 0", 'rgba': "1 0 0 1", 'size': "0.0049535 0.059645 0.1375", 'type': "box"},
                                                            {'condim': "4", 'friction': "1", 'name': "right_toe", 'pos': "0.2195 0 -0.1537", 'quat': "0.6018 0 -0.7986 0", 'rgba': "1 0 0 1", 'size': "0.0049535 0.059645 0.022", 'type': "box"},
                                                            {'condim': "4", 'friction': "1", 'name': "right_heel", 'pos': "-0.0940 0 -0.1566", 'quat': "0.7716 0 -0.6361 0", 'rgba': "1 0 0 1", 'size': "0.0049535 0.059645 0.0165", 'type': "box"}
                                                        ],
                                                        'sites': [
                                                            {'name': "RightHenkeAnkleSite", 'pos': "0 0 0"},
                                                            {'name': "RightHenkeAnkleFront", 'pos': "0.033 -0.1 0"},
                                                            {'name': "RightHenkeAnkleBack", 'pos': "-0.033 -0.1 0"},
                                                            {'name': "RightAnkleFront", 'pos': "0.025 0 -0.1"},
                                                            {'name': "RightAnkleBack", 'pos': "-0.04 0 -0.1"},
                                                            {'name': "right_foot_imu", 'pos': "-0.036966 0.015 -0.13705", 'quat': "-1.6377e-07 -0.94552 -5.639e-08 0.32557", 'size': ".01"},
                                                            {'group': "2", 'name': "opto5", 'pos': "0.17825 0.0285 -0.1598", 'quat': "1 0 0 1", 'size': "0.05"},
                                                            {'group': "2", 'name': "opto6", 'pos': "0.17825 -0.0285 -0.1598", 'quat': "1 0 0 1", 'size': "0.05"},
                                                            {'group': "2", 'name': "opto7", 'pos': "-0.063753 0.02625 -0.1598", 'quat': "1 0 0 1", 'size': "0.05"},
                                                            {'group': "2", 'name': "opto8", 'pos': "-0.063753 -0.02625 -0.1598", 'quat': "1 0 0 1", 'size': "0.05"}
                                                        ]
                                                    }
                                                ]
                                            }
                                        ]
                                    }
                                ]
                            }
                        ]
                    }
                ]
            }
        ]
    }
  ]

        




# Add all bodies to the worldbody
for body in bodies_data:
    add_body(worldbody_node, body)


# Function to add bodies and their nested elements
def add_body(parent_node, body_data):
    body_node = ET.SubElement(parent_node, 'body', name=body_data['name'], pos=body_data['pos'])
    if 'quat' in body_data:
        body_node.set('quat', body_data['quat'])
    if 'childclass' in body_data:
        body_node.set('childclass', body_data['childclass'])
    
    for site in body_data.get('sites', []):
        ET.SubElement(body_node, 'site', site)
    
    for inertial in body_data.get('inertials', []):
        ET.SubElement(body_node, 'inertial', inertial)
    
    for joint in body_data.get('joints', []):
        ET.SubElement(body_node, 'joint', joint)
    
    for geom in body_data.get('geoms', []):
        ET.SubElement(body_node, 'geom', geom)
    
    for child_body in body_data.get('bodies', []):
        add_body(body_node, child_body)

# Add <worldbody> element
worldbody_node = ET.SubElement(root, 'worldbody')

# Define the <geom> element in <worldbody>
terrain_geom = {
    'conaffinity': "15", 'condim': "3", 'friction': "1", 'material': "plane", 'name': "plane",
    'rgba': ".9 .9 .9 1", 'size': "10 5 0.1", 'type': "plane"
}
ET.SubElement(worldbody_node, 'geom', terrain_geom)

# Define all bodies and nested structures
bodies_data = [
    {
        'name': "pelvis",
        'pos':  " ".join(map(str,Muscletorso_pos)),
        'quat': "0.707107 0.707107 0 0",
        'childclass': "myolegs",
        
        'inertials': [
            {'pos': "-0.07 -0.03 0", 'mass': " ".join(map(str,Muscletorso_mass)), 'diaginertia': "0.0622075 0.0532711 0.0299242"}
        ],
        'geoms': [
            {'mesh': "r_pelvis", 'name': "r_pelvis", 'type': "mesh"},
            {'mesh': "l_pelvis", 'name': "l_pelvis", 'type': "mesh"},
            {'mesh': "sacrum", 'name': "sacrum", 'type': "mesh"},
            {'name': "Gmax1_at_pelvis_r_wrap", 'pos': "-0.077 -0.099 0.061", 'quat': "0.931256 -0.288071 0.213142 -0.0659324", 'class': "wrap", 'size': "0.04 0.075"},
            {'name': "Gmax2_at_pelvis_r_wrap", 'pos': "-0.08 -0.083 0.068", 'quat': "0.912872 -0.359331 0.180301 -0.0709714", 'class': "wrap", 'size': "0.04 0.05"},
            {'name': "Gmax3_at_pelvis_r_wrap", 'pos': "-0.083 -0.088 0.068", 'quat': "0.99875 -0.0499792 0 0", 'class': "wrap", 'size': "0.04 0.05"},
            {'name': "Gmax1_at_pelvis_l_wrap", 'pos': "-0.077 -0.099 -0.061", 'quat': "0.931256 0.288071 -0.213142 -0.0659324", 'class': "wrap", 'size': "0.04 0.075"},
            {'name': "Gmax2_at_pelvis_l_wrap", 'pos': "-0.08 -0.083 -0.068", 'quat': "0.912872 0.359331 -0.180301 -0.0709714", 'class': "wrap", 'size': "0.04 0.05"},
            {'name': "Gmax3_at_pelvis_l_wrap", 'pos': "-0.083 -0.088 -0.068", 'quat': "0.99875 0.0499792 0 0", 'class': "wrap", 'size': "0.04 0.05"},
            {'name': "PS_at_brim_r_wrap", 'pos': "-0.074 -0.06 0.0656", 'quat': "0.981103 -0.13006 -0.127199 0.0658971", 'class': "wrap", 'size': "0.05", 'type': "sphere"},
            {'name': "PS_at_brim_l_wrap", 'pos': "-0.074 -0.06 -0.0656", 'quat': "0.981103 0.13006 0.127199 0.0658971", 'class': "wrap", 'size': "0.05", 'type': "sphere"},
            {'name': "IL_at_brim_r_wrap", 'pos': "-0.071 -0.065 0.0756", 'quat': "0.874257 -0.193832 -0.0376181 0.443495", 'class': "wrap", 'size': "0.0549", 'type': "sphere"},
            {'name': "IL_at_brim_l_wrap", 'pos': "-0.071 -0.065 -0.0756", 'quat': "0.874257 0.193832 0.0376181 0.443495", 'class': "wrap", 'size': "0.0549", 'type': "sphere"}
        ],
        'sites': [
            {'name': "pelvislocation", 'pos': "0 0 0"},
            {'name': "pelvis", 'class': "myo_leg_marker"},
            {'name': "addbrev_r-P1", 'pos': "-0.0191 -0.094 0.0154"},
            {'name': "addlong_r-P1", 'pos': "-0.0076 -0.0889 0.0189"},
            {'name': "addmagDist_r-P1", 'pos': "-0.074 -0.1277 0.0398"},
            {'name': "addmagIsch_r-P1", 'pos': "-0.0896 -0.1298 0.0417"},
            {'name': "addmagMid_r-P1", 'pos': "-0.0527 -0.1208 0.0285"},
            {'name': "addmagProx_r-P1", 'pos': "-0.031 -0.1076 0.0136"},
            {'name': "bflh_r-P1", 'pos': "-0.104 -0.1191 0.0586"},
            {'name': "glmax1_r-P1", 'pos': "-0.1231 0.0345 0.0563"},
            {'name': "glmax1_r-P2", 'pos': "-0.1257 -0.0242 0.0779"},
            {'name': "glmax2_r-P1", 'pos': "-0.1317 0.0087 0.0462"},
            {'name': "glmax2_r-P2", 'pos': "-0.1344 -0.0609 0.0813"},
            {'name': "glmax3_r-P1", 'pos': "-0.13 -0.0525 0.009"},
            {'name': "glmax3_r-P2", 'pos': "-0.1273 -0.1263 0.0435"},
            {'name': "glmed1_r-P1", 'pos': "-0.0445 0.0245 0.1172"},
            {'name': "glmed2_r-P1", 'pos': "-0.085 0.0316 0.0675"},
            {'name': "glmed3_r-P1", 'pos': "-0.1152 -0.0073 0.0526"},
            {'name': "glmin1_r-P1", 'pos': "-0.0464 -0.0149 0.1042"},
            {'name': "glmin2_r-P1", 'pos': "-0.0616 -0.0142 0.0971"},
            {'name': "glmin3_r-P1", 'pos': "-0.0789 -0.0155 0.0798"},
            {'name': "grac_r-P1", 'pos': "-0.0474 -0.1293 0.0246"},
            {'name': "iliacus_r-P1", 'pos': "-0.0605 0.0309 0.0843"},
            {'name': "iliacus_r-P2", 'pos': "-0.0135 -0.0557 0.0756"},
            {'name': "piri_r-P1", 'pos': "-0.1018 -0.0065 0.0135"},
            {'name': "piri_r-P2", 'pos': "-0.102 -0.0307 0.0609"},
            {'name': "psoas_r-P1", 'pos': "-0.0606 0.062 0.039"},
            {'name': "psoas_r-P2", 'pos': "-0.0205 -0.0654 0.0656"},
            {'name': "recfem_r-P1", 'pos': "-0.024 -0.0388 0.0933"},
            {'name': "sart_r-P1", 'pos': "-0.0195 -0.0156 0.1056"},
            {'name': "semimem_r-P1", 'pos': "-0.0987 -0.114 0.0614"},
            {'name': "semiten_r-P1", 'pos': "-0.1038 -0.1253 0.0515"},
            {'name': "tfl_r-P1", 'pos': "-0.0311 0.0214 0.1241"},
            {'name': "addbrev_l-P1", 'pos': "-0.0191 -0.094 -0.0154"},
            {'name': "addlong_l-P1", 'pos': "-0.0076 -0.0889 -0.0189"},
            {'name': "addmagDist_l-P1", 'pos': "-0.074 -0.1277 -0.0398"},
            {'name': "addmagIsch_l-P1", 'pos': "-0.0896 -0.1298 -0.0417"},
            {'name': "addmagMid_l-P1", 'pos': "-0.0527 -0.1208 -0.0285"},
            {'name': "addmagProx_l-P1", 'pos': "-0.031 -0.1076 -0.0136"},
            {'name': "bflh_l-P1", 'pos': "-0.104 -0.1191 -0.0586"},
            {'name': "glmax1_l-P1", 'pos': "-0.1231 0.0345 -0.0563"},
            {'name': "glmax1_l-P2", 'pos': "-0.1257 -0.0242 -0.0779"},
            {'name': "glmax2_l-P1", 'pos': "-0.1317 0.0087 -0.0462"},
            {'name': "glmax2_l-P2", 'pos': "-0.1344 -0.0609 -0.0813"},
            {'name': "glmax3_l-P1", 'pos': "-0.13 -0.0525 -0.009"},
            {'name': "glmax3_l-P2", 'pos': "-0.1273 -0.1263 -0.0435"},
            {'name': "glmed1_l-P1", 'pos': "-0.0445 0.0245 -0.1172"},
            {'name': "glmed2_l-P1", 'pos': "-0.085 0.0316 -0.0675"},
            {'name': "glmed3_l-P1", 'pos': "-0.1152 -0.0073 -0.0526"},
            {'name': "glmin1_l-P1", 'pos': "-0.0464 -0.0149 -0.1042"},
            {'name': "glmin2_l-P1", 'pos': "-0.0616 -0.0142 -0.0971"},
            {'name': "glmin3_l-P1", 'pos': "-0.0789 -0.0155 -0.0798"},
            {'name': "grac_l-P1", 'pos': "-0.0474 -0.1293 -0.0246"},
            {'name': "iliacus_l-P1", 'pos': "-0.0605 0.0309 -0.0843"},
            {'name': "iliacus_l-P2", 'pos': "-0.0135 -0.0557 -0.0756"},
            {'name': "piri_l-P1", 'pos': "-0.1018 -0.0065 -0.0135"},
            {'name': "piri_l-P2", 'pos': "-0.102 -0.0307 -0.0609"},
            {'name': "psoas_l-P1", 'pos': "-0.0606 0.062 -0.039"},
            {'name': "psoas_l-P2", 'pos': "-0.0205 -0.0654 -0.0656"},
            {'name': "recfem_l-P1", 'pos': "-0.024 -0.0388 -0.0933"},
            {'name': "sart_l-P1", 'pos': "-0.0195 -0.0156 -0.1056"},
            {'name': "semimem_l-P1", 'pos': "-0.0987 -0.114 -0.0614"},
            {'name': "semiten_l-P1", 'pos': "-0.1038 -0.1253 -0.0515"},
            {'name': "tfl_l-P1", 'pos': "-0.0311 0.0214 -0.1241"},
            {'name': "RASI", 'pos': "0.0095 0.0181 0.1285"},
            {'name': "LASI", 'pos': "0.0095 0.0181 -0.1285"},
            {'name': "RPSI", 'pos': "-0.155 0.035 0.045"},
            {'name': "LPSI", 'pos': "-0.155 0.035 -0.045"},
            {'name': "Gmax1_at_pelvis_r_site_glmax1_r_side", 'pos': "-0.0929654 -0.0868446 0.124923"},
            {'name': "Gmax2_at_pelvis_r_site_glmax2_r_side", 'pos': "-0.13018 -0.1232159 0.1219181"},
            {'name': "Gmax3_at_pelvis_r_site_glmax3_r_side", 'pos': "-0.0530013 -0.127251 0.0521302"},
            {'name': "Gmax1_at_pelvis_l_site_glmax1_l_side", 'pos': "-0.0929654 -0.0868446 -0.124923"},
            {'name': "Gmax2_at_pelvis_l_site_glmax2_l_side", 'pos': "-0.13018 -0.1232159 -0.1219181"},
            {'name': "Gmax3_at_pelvis_l_site_glmax3_l_side", 'pos': "-0.0530013 -0.127251 -0.0521302"},
            {'name': "PS_at_brim_r_site_psoas_r_side", 'pos': "-0.0209166 -0.0856869 0.0790765"},
            {'name': "PS_at_brim_l_site_psoas_l_side", 'pos': "-0.0209166 -0.0856869 -0.0790765"},
            {'name': "IL_at_brim_r_site_iliacus_r_side", 'pos': "-0.02526034 -0.0256276 0.0928915"},
            {'name': "IL_at_brim_l_site_iliacus_l_side", 'pos': "-0.00526034 -0.0756276 -0.0828915"}
        ],
        'bodies': [
            {
                'name': "femur_r",
                'pos': " ".join(map(str,Musclefemur_r_pos)),
                'sites': [{'name': "hip_r_location", 'pos': "0 0 0"}, {'name': "hip_r", 'class': "myo_leg_marker"}],
                'inertials': [{'pos': "0 -0.195 -0.0005", 'quat': "0.708013 -0.7062 0 0", 'mass': " ".join(map(str,Musclefemur_r_mass)), 'diaginertia': "0.1694 0.1694 0.0245269"}],
                'joints': [
                    {'axis': "0 0 1", 'name': "hip_flexion_r", 'pos': "0 0 0", 'range': "-0.523599 2.0944"},
                    {'axis': "1 0 0", 'name': "hip_adduction_r", 'pos': "0 0 0", 'range': "-0.872665 0.523599"},
                    {'axis': "0 1 0", 'name': "hip_rotation_r", 'pos': "0 0 0", 'range': "-0.698132 0.698132"}
                ],
                'geoms': [
                    {'mesh': "r_femur", 'name': "r_femur", 'type': "mesh"},
                    {'name': "Gastroc_at_condyles_r_wrap", 'pos': "0.005 -0.41 0", 'class': "wrap", 'size': "0.025 0.05"},
                    {'name': "KnExt_at_fem_r_wrap", 'pos': "0.00358828 -0.402732 0.00209111", 'quat': "0.999192 -0.0311532 0.025365 -0.00079084", 'class': "wrap", 'size': "0.025 0.05"},
                    {'name': "AB_at_femshaft_r_wrap", 'pos': "0.0146434 -0.112595 0.023365", 'quat': "0.671362 0.735248 0.0628354 0.0688147", 'class': "wrap", 'size': "0.0165 0.035"},
                    {'name': "AL_at_femshaft_r_wrap", 'pos': "0.0307327 -0.231909 0.0151137", 'quat': "0.629067 0.774355 0.0429971 0.0529276", 'class': "wrap", 'size': "0.0201 0.05"},
                    {'name': "AMprox_at_femshaft_r_wrap", 'pos': "0.00518299 -0.0728948 0.025403", 'quat': "0.689646 0.718132 0.0645174 0.0671823", 'class': "wrap", 'size': "0.0211 0.035"},
                    {'name': "AMmid_at_femshaft_r_wrap", 'pos': "0.0230125 -0.160711 0.0205842", 'quat': "0.690996 0.719631 0.0472547 0.0492129", 'class': "wrap", 'size': "0.0214 0.06"},
                    {'name': "AMdist_at_femshaft_r_wrap", 'pos': "0.0316065 -0.260736 0.0093646", 'quat': "0.652657 0.751902 0.061082 0.0703703", 'class': "wrap", 'size': "0.0218 0.1"},
                    {'name': "AMisch_at_condyles_r_wrap", 'pos': "-0.0226511 -0.376831 -0.00315437", 'quat': "0.638263 0.734777 -0.150578 -0.173347", 'class': "wrap", 'size': "0.04 0.12"},
                    {'name': "PECT_at_femshaft_r_wrap", 'pos': "0.00608573 -0.0845029 0.0304405", 'quat': "0.610649 0.779832 0.0849157 0.108442", 'class': "wrap", 'size': "0.015 0.025"}
                ],
                'sites': [
                    {'name': "hip_r_location", 'pos': "0 0 0"},
                    {'name': "addbrev_r-P2", 'pos': "-0.002 -0.118 0.0249"},
                    {'name': "addlong_r-P2", 'pos': "0.0113 -0.2394 0.0158"},
                    {'name': "addmagDist_r-P2", 'pos': "0.0112 -0.2625 0.0193"},
                    {'name': "addmagIsch_r-P2", 'pos': "0.0048 -0.388 -0.0327"},
                    {'name': "addmagMid_r-P2", 'pos': "0.0024 -0.1624 0.0292"},
                    {'name': "addmagProx_r-P2", 'pos': "-0.0153 -0.0789 0.032"},
                    {'name': "bfsh_r-P1", 'pos': "0.005 -0.2111 0.0234"},
                    {'name': "gaslat_r-P1", 'pos': "-0.003 -0.3814 0.0277"},
                    {'name': "gasmed_r-P1", 'pos': "0.008 -0.3788 -0.0208"},
                    {'name': "glmax1_r-P3", 'pos': "-0.0444 -0.0326 0.0302"},
                    {'name': "glmax1_r-P4", 'pos': "-0.0277 -0.0566 0.047"},
                    {'name': "glmax2_r-P3", 'pos': "-0.045 -0.0584 0.0252"},
                    {'name': "glmax2_r-P4", 'pos': "-0.0156 -0.1016 0.0419"},
                    {'name': "glmax3_r-P3", 'pos': "-0.0281 -0.1125 0.0094"},
                    {'name': "glmax3_r-P4", 'pos': "-0.006 -0.1419 0.0411"},
                    {'name': "glmed1_r-P2", 'pos': "-0.0218 -0.0117 0.0555"},
                    {'name': "glmed2_r-P2", 'pos': "-0.0258 -0.0058 0.0527"},
                    {'name': "glmed3_r-P2", 'pos': "-0.0309 -0.0047 0.0518"},
                    {'name': "glmin1_r-P2", 'pos': "-0.0072 -0.0104 0.056"},
                    {'name': "glmin2_r-P2", 'pos': "-0.0096 -0.0104 0.056"},
                    {'name': "glmin3_r-P2", 'pos': "-0.0135 -0.0083 0.055"},
                    {'name': "iliacus_r-P3", 'pos': "-0.0023 -0.0565 0.0139"},
                    {'name': "iliacus_r-P4", 'pos': "-0.0122 -0.0637 0.0196"},
                    {'name': "piri_r-P3", 'pos': "-0.0148 -0.0036 0.0437"},
                    {'name': "psoas_r-P3", 'pos': "-0.0132 -0.0467 0.0046"},
                    {'name': "psoas_r-P4", 'pos': "-0.0235 -0.0524 0.0088"},
                    {'name': "sart_r-P2", 'pos': "-0.003 -0.3568 -0.0421"},
                    {'name': "tfl_r-P2", 'pos': "0.0294 -0.0995 0.0597"},
                    {'name': "tfl_r-P3", 'pos': "0.0107 -0.405 0.0324"},
                    {'name': "vasint_r-P1", 'pos': "0.029 -0.1924 0.031"},
                    {'name': "vasint_r-P2", 'pos': "0.0335 -0.2084 0.0285"},
                    {'name': "vaslat_r-P1", 'pos': "0.0048 -0.1854 0.0349"},
                    {'name': "vaslat_r-P2", 'pos': "0.0269 -0.2591 0.0409"},
                    {'name': "vasmed_r-P1", 'pos': "0.014 -0.2099 0.0188"},
                    {'name': "vasmed_r-P2", 'pos': "0.0356 -0.2769 0.0009"},
                    {'name': "RHJC", 'pos': "0 0 0"},
                    {'name': "RTH1", 'pos': "0.018 -0.15 0.064"},
                    {'name': "RTH2", 'pos': "0.08 -0.23 0.0047"},
                    {'name': "RTH3", 'pos': "0.01 -0.3 0.06"},
                    {'name': "RLFC", 'pos': "0 -0.404 0.05"},
                    {'name': "RMFC", 'pos': "0 -0.404 -0.05"},
                    {'name': "KnExt_at_fem_r_site_recfem_r_side", 'pos': "0.028412 -0.418795 -0.0326861"},
                    {'name': "KnExt_at_fem_r_site_vasint_r_side", 'pos': "0.0140493 -0.375075 -0.00469312"},
                    {'name': "KnExt_at_fem_r_site_vaslat_r_side", 'pos': "0.0164816 -0.378983 -0.0366504"},
                    {'name': "KnExt_at_fem_r_site_vasmed_r_side", 'pos': "0.0179815 -0.374402 0.023524"},
                    {'name': "AB_at_femshaft_r_site_addbrev_r_side", 'pos': "-0.00249969 -0.126567 0.0261656"},
                    {'name': "AL_at_femshaft_r_site_addlong_r_side", 'pos': "0.0113183 -0.263228 0.00405212"},
                    {'name': "AMprox_at_femshaft_r_site_addmagProx_r_side", 'pos': "-0.0232677 -0.056978 0.0222299"},
                    {'name': "AMmid_at_femshaft_r_site_addmagMid_r_side", 'pos': "-0.0100694 -0.108641 0.0230602"},
                    {'name': "AMdist_at_femshaft_r_site_addmagDist_r_side", 'pos': "0.0146959 -0.298529 0.0158276"},
                    {'name': "AMisch_at_condyles_r_site_addmagIsch_r_side", 'pos': "-0.0360341 -0.49032 -0.0446456"}
                ],
                'bodies': [
                    {
                        'name': "tibia_r",
                        'pos': " ".join(map(str,Muscletibia_r_pos)),
                        'sites': [{'name': "knee_r_location", 'pos': "0 0 0"}, {'name': "knee_r", 'class': "myo_leg_marker"}],
                        'inertials': [{'pos': "-0.005 -0.175 0.0025", 'quat': "0.70204 -0.711847 0.0203385 0", 'mass': " ".join(map(str,Muscletibia_r_mass)), 'diaginertia': "0.0771589 0.0771589 0.00690387"}],
                        'joints': [
                            {'axis': "0.992246 0.123982 -0.00878916", 'name': "knee_angle_r_translation2", 'pos': "0 0 0", 'range': "7.69254e-11 0.006792", 'type': "slide"},
                            {'axis': "-0.124293 0.989762 -0.0701648", 'name': "knee_angle_r_translation1", 'pos': "0 0 0", 'range': "9.53733e-08 0.00159883", 'type': "slide"},
                            {'axis': "-3.98373e-10 -0.0707131 -0.997497", 'name': "knee_angle_r", 'pos': "0 0 0", 'range': "0 2.0944"},
                            {'axis': "0.992246 0.123982 -0.00878916", 'name': "knee_angle_r_rotation2", 'pos': "0 0 0", 'range': "-0.00167821 0.0335354"},
                            {'axis': "-0.124293 0.989762 -0.0701648", 'name': "knee_angle_r_rotation3", 'pos': "0 0 0", 'range': "1.08939e-08 0.262788"}
                        ],
                        'geoms': [
                            {'mesh': "r_tibia", 'name': "r_tibia", 'type': "mesh"},
                            {'mesh': "r_fibula", 'name': "r_fibula", 'type': "mesh"},
                            {'name': "GasLat_at_shank_r_wrap", 'pos': "-0.0074 -0.074 -0.0033", 'quat': "-0.0298211 0.737282 0.655511 -0.160722", 'class': "wrap", 'size': "0.055 0.05"},
                            {'name': "GasMed_at_shank_r_wrap", 'pos': "-0.0074 -0.074 -0.0033", 'quat': "0.073733 0.735403 0.67187 -0.048347", 'class': "wrap", 'size': "0.055 0.05"},
                            {'name': "GR_at_condyles_r_wrap", 'pos': "-0.003 -0.02 0", 'quat': "0.980067 0 -0.198669 0", 'class': "wrap", 'size': "0.036 0.05"},
                            {'name': "SM_at_condyles_r_wrap", 'pos': "-0.001 -0.02 0", 'quat': "0.99875 0 -0.0499792 0", 'class': "wrap", 'size': "0.0352 0.05"},
                            {'name': "ST_at_condyles_r_wrap", 'pos': "-0.002 -0.0205 0", 'quat': "0.995004 0 -0.0998334 0", 'class': "wrap", 'size': "0.0425 0.05"},
                            {'name': "BF_at_gastroc_r_wrap", 'pos': "-0.058 -0.06 0", 'class': "wrap", 'size': "0.03 0.075"}
                        ],
                        'sites': [
                            {'name': "knee_r_location", 'pos': "0 0 0"}, {'name': "knee_r", 'class': "myo_leg_marker"},
                            {'name': "bflh_r-P2", 'pos': "-0.0337 -0.035 0.0253"},
                            {'name': "bflh_r-P3", 'pos': "-0.0287 -0.0455 0.0303"},
                            {'name': "bfsh_r-P2", 'pos': "-0.0301 -0.0419 0.0318"},
                            {'name': "edl_r-P1", 'pos': "-0.016 -0.1157 0.0205"},
                            {'name': "edl_r-P2", 'pos': "0.0164 -0.376 0.0112"},
                            {'name': "ehl_r-P1", 'pos': "-0.014 -0.155 0.0189"},
                            {'name': "ehl_r-P2", 'pos': "0.0071 -0.2909 0.0164"},
                            {'name': "ehl_r-P3", 'pos': "0.02 -0.3693 -0.0028"},
                            {'name': "fdl_r-P1", 'pos': "-0.0023 -0.1832 -0.0018"},
                            {'name': "fdl_r-P2", 'pos': "-0.0176 -0.3645 -0.0124"},
                            {'name': "fhl_r-P1", 'pos': "-0.031 -0.2163 0.02"},
                            {'name': "fhl_r-P2", 'pos': "-0.0242 -0.3671 -0.0076"},
                            {'name': "grac_r-P2", 'pos': "-0.0184 -0.0476 -0.0296"},
                            {'name': "grac_r-P3", 'pos': "0.0018 -0.0696 -0.0157"},
                            {'name': "perbrev_r-P1", 'pos': "-0.0243 -0.2532 0.0251"},
                            {'name': "perbrev_r-P2", 'pos': "-0.0339 -0.3893 0.0249"},
                            {'name': "perbrev_r-P3", 'pos': "-0.0285 -0.4004 0.0255"},
                            {'name': "perlong_r-P1", 'pos': "-0.02 -0.1373 0.0282"},
                            {'name': "perlong_r-P2", 'pos': "-0.0317 -0.39 0.0237"},
                            {'name': "perlong_r-P3", 'pos': "-0.0272 -0.4014 0.024"},
                            {'name': "recfem_r-P5", 'pos': "0.0326 -0.0631 -0.0005"},
                            {'name': "sart_r-P3", 'pos': "-0.0251 -0.0401 -0.0365"},
                            {'name': "sart_r-P4", 'pos': "-0.0159 -0.0599 -0.0264"},
                            {'name': "sart_r-P5", 'pos': "0.0136 -0.081 -0.0026"},
                            {'name': "semimem_r-P2", 'pos': "-0.029 -0.0417 -0.0196"},
                            {'name': "semiten_r-P2", 'pos': "-0.0312 -0.0508 -0.0229"},
                            {'name': "semiten_r-P3", 'pos': "0.0019 -0.0773 -0.0117"},
                            {'name': "soleus_r-P1", 'pos': "-0.0076 -0.0916 0.0098"},
                            {'name': "tfl_r-P4", 'pos': "0.0108 -0.041 0.0346"},
                            {'name': "tibant_r-P1", 'pos': "0.0154 -0.1312 0.0162"},
                            {'name': "tibant_r-P2", 'pos': "0.0251 -0.1906 0.0128"},
                            {'name': "tibant_r-P3", 'pos': "0.0233 -0.3659 -0.0132"},
                            {'name': "tibpost_r-P1", 'pos': "-0.0041 -0.1304 0.0103"},
                            {'name': "tibpost_r-P2", 'pos': "-0.0164 -0.3655 -0.0175"},
                            {'name': "vasint_r-P5", 'pos': "0.0326 -0.0632 0.0004"},
                            {'name': "vaslat_r-P5", 'pos': "0.0325 -0.0634 0.0051"},
                            {'name': "vasmed_r-P5", 'pos': "0.0319 -0.0636 -0.0068"},
                            {'name': "RKJC", 'pos': "0.0017 -0.0024 -0.0085"},
                            {'name': "RTB1", 'pos': "-0.0017 -0.1565 0.0492"},
                            {'name': "RTB2", 'pos': "0.037 -0.2301 -0.0039"},
                            {'name': "RTB3", 'pos': "0.0114 -0.2952 0.0554"},
                            {'name': "RLMAL", 'pos': "-0.005 -0.3888 0.053"},
                            {'name': "RMMAL", 'pos': "0.006 -0.3888 -0.038"},
                            {'name': "R_tibial_plateau", 'pos': "-0.0081 -0.017 -0.0015"},
                            {'name': "GasLat_at_shank_r_site_gaslat_r_side", 'pos': "-0.0512647 -0.0235815 -0.0514609"},
                            {'name': "GasMed_at_shank_r_site_gasmed_r_side", 'pos': "-0.0226964 -0.1301 0.0498879"},
                            {'name': "GR_at_condyles_r_site_grac_r_side", 'pos': "-0.0513804 -0.042587 0.0198669"},
                            {'name': "SM_at_condyles_r_site_semimem_r_side", 'pos': "-0.0387876 -0.035171 -0.0182577"},
                            {'name': "ST_at_condyles_r_site_semiten_r_side", 'pos': "-0.0428471 -0.0468901 -0.0181631"},
                            {'name': "BF_at_gastroc_r_site_bfsh_r_side", 'pos': "-0.0687049 -0.0256284 0.0426731"}
                        ],
                        'bodies': [
                            {
                                'name': "talus_r",
                                'pos': " ".join(map(str,Muscletalus_r_pos)),
                                'joints': [{'axis': "-0.105014 -0.174022 0.979126", 'name': "ankle_angle_r", 'pos': "0 0 0", 'range': "-0.698132 0.523599"}],
                                'geoms': [{'mesh': "r_talus", 'name': "r_talus", 'type': "mesh"}],
                                'sites': [
                                    {'name': "RAJC", 'pos': "0 0 0"},
                                    {'name': "ankle_r_location", 'pos': "0 0 0"}, {'name': "ankle_r", 'class': "myo_leg_marker"}
                                    ],
                                'bodies': [
                                    {
                                        'name': "calcn_r",
                                        'pos': " ".join(map(str,Musclecalcn_r_pos)),
                                        'inertials': [{'pos': "0.0821377 0.0108024 -0.000944392", 'quat': "0.502987 0.541341 0.493601 0.458598", 'mass': " ".join(map(str,Musclecalcn_r_mass)), 'diaginertia': "0.00313636 0.00297113 0.000941737"}],
                                        'joints': [{'axis': "0.78718 0.604747 -0.120949", 'name': "subtalar_angle_r", 'pos': "0 0 0", 'range': "-0.349066 0.349066"}],
                                        'geoms': [{'mesh': "r_foot", 'name': "r_foot", 'type': "mesh"}],
                                        'sites': [
                                            {'name': "edl_r-P3", 'pos': "0.0919 0.036 0.0008"},
                                            {'name': "edl_r-P4", 'pos': "0.1616 0.0055 0.013"},
                                            {'name': "ehl_r-P4", 'pos': "0.097 0.0389 -0.0211"},
                                            {'name': "ehl_r-P5", 'pos': "0.1293 0.0309 -0.0257"},
                                            {'name': "ehl_r-P6", 'pos': "0.1734 0.0139 -0.028"},
                                            {'name': "fdl_r-P3", 'pos': "0.0436 0.0315 -0.028"},
                                            {'name': "fdl_r-P4", 'pos': "0.0708 0.0176 -0.0263"},
                                            {'name': "fdl_r-P5", 'pos': "0.1658 -0.0081 0.0116"},
                                            {'name': "fhl_r-P3", 'pos': "0.0374 0.0276 -0.0241"},
                                            {'name': "fhl_r-P4", 'pos': "0.1038 0.0068 -0.0256"},
                                            {'name': "fhl_r-P5", 'pos': "0.1726 -0.0053 -0.0269"},
                                            {'name': "gaslat_r-P2", 'pos': "0.0044 0.031 -0.0053"},
                                            {'name': "gasmed_r-P2", 'pos': "0.0044 0.031 -0.0053"},
                                            {'name': "perbrev_r-P4", 'pos': "0.0471 0.027 0.0233"},
                                            {'name': "perbrev_r-P5", 'pos': "0.0677 0.0219 0.0343"},
                                            {'name': "perlong_r-P4", 'pos': "0.0438 0.023 0.0221"},
                                            {'name': "perlong_r-P5", 'pos': "0.0681 0.0106 0.0284"},
                                            {'name': "perlong_r-P6", 'pos': "0.0852 0.0069 0.0118"},
                                            {'name': "perlong_r-P7", 'pos': "0.1203 0.0085 -0.0184"},
                                            {'name': "soleus_r-P2", 'pos': "0.0044 0.031 -0.0053"},
                                            {'name': "tibant_r-P4", 'pos': "0.1166 0.0178 -0.0305"},
                                            {'name': "tibpost_r-P3", 'pos': "0.0417 0.0334 -0.0286"},
                                            {'name': "tibpost_r-P4", 'pos': "0.0772 0.0159 -0.0281"},
                                            {'name': "RCAL", 'pos': "-0.025 0.02 -0.005"},
                                            {'name': "RTOE", 'pos': "0.205 0.0297 -0.03"},
                                            {'name': "RMT5", 'pos': "0.145 0.0249 0.059"},
                                            {'name': "r_foot_touch", 'type': "box", 'pos': "0.09 -.01 0.0", 'size': ".1 .01 .055", 'euler': "0 0 0", 'class': "myo_leg_touch"}
                                        ],
                                        'bodies': [
                                            {
                                                'name': "toes_r",
                                                'pos': " ".join(map(str,Muscletoes_r_pos)),
                                                'joints': [{'axis': "0.580954 0 -0.813936", 'name': "mtp_angle_r", 'pos': "0 0 0", 'range': "-0.523599 0.523599"}],
                                                'geoms': [{'mesh': "r_bofoot", 'name': "r_bofoot", 'type': "mesh"}],
                                                'sites': [
                                                    {'name': "toe_r_location", 'pos': "0 0 0"}, {'name': "toe_r", 'class': "myo_leg_marker"},
                                                    {'name': "r_toes_touch", 'type': "box", 'pos': "0.0275 -.01 0", 'size': ".04 .01 .0675", 'euler': "0 -.7 0", 'class': "myo_leg_touch"},
                                                    {'name': "edl_r-P5", 'pos': "0.0003 0.0047 0.0153"},
                                                    {'name': "edl_r-P6", 'pos': "0.0443 -0.0004 0.025"},
                                                    {'name': "ehl_r-P7", 'pos': "0.0298 0.0041 -0.0245"},
                                                    {'name': "ehl_r-P8", 'pos': "0.0563 0.0034 -0.0186"},
                                                    {'name': "fdl_r-P6", 'pos': "-0.0019 -0.0078 0.0147"},
                                                    {'name': "fdl_r-P7", 'pos': "0.0285 -0.0071 0.0215"},
                                                    {'name': "fdl_r-P8", 'pos': "0.0441 -0.006 0.0242"},
                                                    {'name': "fhl_r-P6", 'pos': "0.0155 -0.0064 -0.0265"},
                                                    {'name': "fhl_r-P7", 'pos': "0.0562 -0.0102 -0.0181"}
                                                ]
                                            }
                                        ]
                                    }
                                ]
                            }
                        ]
                    },
                    {
                        'name': "patella_r",
                        'pos': " ".join(map(str,Musclepatella_r_pos)),
                        'joints': [
                            {'axis': "0 1 0", 'name': "knee_angle_r_beta_translation2", 'pos': "0 0 0", 'range': "-0.0408267 -0.0108281", 'type': "slide"},
                            {'axis': "1 0 0", 'name': "knee_angle_r_beta_translation1", 'pos': "0 0 0", 'range': "-0.0227731 0.0524192", 'type': "slide"},
                            {'axis': "0 0 1", 'name': "knee_angle_r_beta_rotation1", 'pos': "0 0 0", 'range': "-1.79241 0.010506"}
                        ],
                        'geoms': [{'mesh': "r_patella", 'name': "r_patella", 'type': "mesh"}],
                        'sites': [
                            {'name': "recfem_r-P2", 'pos': "0.01 0.049 0.0007"},
                            {'name': "recfem_r-P3", 'pos': "0.0121 0.0437 -0.001"},
                            {'name': "recfem_r-P4", 'pos': "0.005 0.0025 0"},
                            {'name': "vasint_r-P3", 'pos': "0.0058 0.048 -0.0006"},
                            {'name': "vasint_r-P4", 'pos': "0.005 0.0025 -0.0004"},
                            {'name': "vaslat_r-P3", 'pos': "0.0103 0.0423 0.0141"},
                            {'name': "vaslat_r-P4", 'pos': "0.005 0.0025 0.0073"},
                            {'name': "vasmed_r-P3", 'pos': "0.0063 0.0445 -0.017"},
                            {'name': "vasmed_r-P4", 'pos': "0.005 0.0025 -0.0085"}
                        ]
                    }
                ]
            },
            {
                'name': "femur_l",
                'pos': " ".join(map(str,Musclefemur_l_pos)),
                'inertials': [{'pos': "0 -0.195 0.0005", 'quat': "0.7062 -0.708013 0 0", 'mass':" ".join(map(str,Musclefemur_l_mass)), 'diaginertia': "0.1694 0.1694 0.0245269"}],
                'joints': [
                    {'axis': "0 0 1", 'name': "hip_flexion_l", 'pos': "0 0 0", 'range': "-0.523599 2.0944"},
                    {'axis': "-1 0 0", 'name': "hip_adduction_l", 'pos': "0 0 0", 'range': "-0.872665 0.523599"},
                    {'axis': "0 -1 0", 'name': "hip_rotation_l", 'pos': "0 0 0", 'range': "-0.698132 0.698132"}
                ],
                'geoms': [
                    {'mesh': "l_femur", 'name': "l_femur", 'type': "mesh"},
                    {'name': "Gastroc_at_condyles_l_wrap", 'pos': "0.005 -0.41 0", 'class': "wrap", 'size': "0.025 0.05"},
                    {'name': "KnExt_at_fem_l_wrap", 'pos': "0.00358828 -0.402732 -0.00209111", 'quat': "0.999192 0.0311532 -0.025365 -0.00079084", 'class': "wrap", 'size': "0.025 0.05"},
                    {'name': "AB_at_femshaft_l_wrap", 'pos': "0.0146434 -0.112595 -0.023365", 'quat': "0.671362 -0.735248 -0.0628354 0.0688147", 'class': "wrap", 'size': "0.0165 0.035"},
                    {'name': "AL_at_femshaft_l_wrap", 'pos': "0.0307327 -0.231909 -0.0151137", 'quat': "0.629067 -0.774355 -0.0429971 0.0529276", 'class': "wrap", 'size': "0.0201 0.05"},
                    {'name': "AMprox_at_femshaft_l_wrap", 'pos': "0.00518299 -0.0728948 -0.025403", 'quat': "0.689646 -0.718132 -0.0645174 0.0671823", 'class': "wrap", 'size': "0.0211 0.035"},
                    {'name': "AMmid_at_femshaft_l_wrap", 'pos': "0.0230125 -0.160711 -0.0205842", 'quat': "0.690996 -0.719631 -0.0472547 0.0492129", 'class': "wrap", 'size': "0.0214 0.06"},
                    {'name': "AMdist_at_femshaft_l_wrap", 'pos': "0.0316065 -0.260736 -0.0093646", 'quat': "0.652657 -0.751902 -0.061082 0.0703703", 'class': "wrap", 'size': "0.0218 0.1"},
                    {'name': "AMisch_at_condyles_l_wrap", 'pos': "-0.0226511 -0.376831 0.00315437", 'quat': "0.638263 -0.734777 0.150578 -0.173347", 'class': "wrap", 'size': "0.04 0.12"},
                    {'name': "PECT_at_femshaft_l_wrap", 'pos': "0.00608573 -0.0845029 -0.0304405", 'quat': "0.610649 -0.779832 -0.0849157 0.108442", 'class': "wrap", 'size': "0.015 0.025"}
                ],
                'sites': [
                    {'name': "hip_l_location", 'pos': "0 0 0"}, 
                    {'name': "hip_l", 'class': "myo_leg_marker"},
                    {'name': "addbrev_l-P2", 'pos': "-0.002 -0.118 -0.0249"},
                    {'name': "addlong_l-P2", 'pos': "0.0113 -0.2394 -0.0158"},
                    {'name': "addmagDist_l-P2", 'pos': "0.0112 -0.2625 -0.0193"},
                    {'name': "addmagIsch_l-P2", 'pos': "0.0048 -0.388 0.0327"},
                    {'name': "addmagMid_l-P2", 'pos': "0.0024 -0.1624 -0.0292"},
                    {'name': "addmagProx_l-P2", 'pos': "-0.0153 -0.0789 -0.032"},
                    {'name': "bfsh_l-P1", 'pos': "0.005 -0.2111 -0.0234"},
                    {'name': "gaslat_l-P1", 'pos': "-0.003 -0.3814 -0.0277"},
                    {'name': "gasmed_l-P1", 'pos': "0.008 -0.3788 0.0208"},
                    {'name': "glmax1_l-P3", 'pos': "-0.0444 -0.0326 -0.0302"},
                    {'name': "glmax1_l-P4", 'pos': "-0.0277 -0.0566 -0.047"},
                    {'name': "glmax2_l-P3", 'pos': "-0.045 -0.0584 -0.0252"},
                    {'name': "glmax2_l-P4", 'pos': "-0.0156 -0.1016 -0.0419"},
                    {'name': "glmax3_l-P3", 'pos': "-0.0281 -0.1125 -0.0094"},
                    {'name': "glmax3_l-P4", 'pos': "-0.006 -0.1419 -0.0411"},
                    {'name': "glmed1_l-P2", 'pos': "-0.0218 -0.0117 -0.0555"},
                    {'name': "glmed2_l-P2", 'pos': "-0.0258 -0.0058 -0.0527"},
                    {'name': "glmed3_l-P2", 'pos': "-0.0309 -0.0047 -0.0518"},
                    {'name': "glmin1_l-P2", 'pos': "-0.0072 -0.0104 -0.056"},
                    {'name': "glmin2_l-P2", 'pos': "-0.0096 -0.0104 -0.056"},
                    {'name': "glmin3_l-P2", 'pos': "-0.0135 -0.0083 -0.055"},
                    {'name': "iliacus_l-P3", 'pos': "-0.0023 -0.0565 -0.0139"},
                    {'name': "iliacus_l-P4", 'pos': "-0.0122 -0.0637 -0.0196"},
                    {'name': "piri_l-P3", 'pos': "-0.0148 -0.0036 -0.0437"},
                    {'name': "psoas_l-P3", 'pos': "-0.0132 -0.0467 -0.0046"},
                    {'name': "psoas_l-P4", 'pos': "-0.0235 -0.0524 -0.0088"},
                    {'name': "sart_l-P2", 'pos': "-0.003 -0.3568 0.0421"},
                    {'name': "tfl_l-P2", 'pos': "0.0294 -0.0995 -0.0597"},
                    {'name': "tfl_l-P3", 'pos': "0.0107 -0.405 -0.0324"},
                    {'name': "vasint_l-P1", 'pos': "0.029 -0.1924 -0.031"},
                    {'name': "vasint_l-P2", 'pos': "0.0335 -0.2084 -0.0285"},
                    {'name': "vaslat_l-P1", 'pos': "0.0048 -0.1854 -0.0349"},
                    {'name': "vaslat_l-P2", 'pos': "0.0269 -0.2591 -0.0409"},
                    {'name': "vasmed_l-P1", 'pos': "0.014 -0.2099 -0.0188"},
                    {'name': "vasmed_l-P2", 'pos': "0.0356 -0.2769 -0.0009"},
                    {'name': "LHJC", 'pos': "0 0 0"},
                    {'name': "LTH1", 'pos': "0.018 -0.15 -0.064"},
                    {'name': "LTH2", 'pos': "0.08 -0.23 -0.0047"},
                    {'name': "LTH3", 'pos': "0.01 -0.3 -0.06"},
                    {'name': "LLFC", 'pos': "0 -0.404 -0.05"},
                    {'name': "LMFC", 'pos': "0 -0.404 0.05"},
                    {'name': "KnExt_at_fem_l_site_recfem_l_side", 'pos': "0.028412 -0.418795 0.0326861"},
                    {'name': "KnExt_at_fem_l_site_vasint_l_side", 'pos': "0.0140493 -0.375075 0.00469312"},
                    {'name': "KnExt_at_fem_l_site_vaslat_l_side", 'pos': "0.0164816 -0.378983 0.0366504"},
                    {'name': "KnExt_at_fem_l_site_vasmed_l_side", 'pos': "0.0179815 -0.374402 -0.023524"},
                    {'name': "AB_at_femshaft_l_site_addbrev_l_side", 'pos': "-0.00249969 -0.126567 -0.0261656"},
                    {'name': "AL_at_femshaft_l_site_addlong_l_side", 'pos': "0.0113183 -0.263228 -0.00405212"},
                    {'name': "AMprox_at_femshaft_l_site_addmagProx_l_side", 'pos': "-0.0232677 -0.056978 -0.0222299"},
                    {'name': "AMmid_at_femshaft_l_site_addmagMid_l_side", 'pos': "-0.0100694 -0.108641 -0.0230602"},
                    {'name': "AMdist_at_femshaft_l_site_addmagDist_l_side", 'pos': "0.0146959 -0.298529 -0.0158276"},
                    {'name': "AMisch_at_condyles_l_site_addmagIsch_l_side", 'pos': "-0.0360341 -0.49032 0.0446456"}
                ],
                'bodies': [
                    {
                        'name': "tibia_l",
                        'pos': " ".join(map(str,Muscletibia_l_pos)),
                        'inertials': [{'pos': "-0.005 -0.175 -0.0025", 'quat': "0.712137 -0.701754 0.0200501 0", 'mass': " ".join(map(str,Muscletibia_l_mass)), 'diaginertia': "0.0771589 0.0771589 0.00690387"}],
                        'joints': [
                            {'axis': "-0.992246 -0.123982 -0.00878916", 'name': "knee_angle_l_translation2", 'pos': "0 0 0", 'range': "-0.006792 -7.69254e-11", 'type': "slide"},
                            {'axis': "-0.124293 0.989762 0.0701648", 'name': "knee_angle_l_translation1", 'pos': "0 0 0", 'range': "9.53733e-08 0.00159883", 'type': "slide"},
                            {'axis': "3.98373e-10 0.0707131 -0.997497", 'name': "knee_angle_l", 'pos': "0 0 0", 'range': "0 2.0944"},
                            {'axis': "-0.992246 -0.123982 -0.00878916", 'name': "knee_angle_l_rotation2", 'pos': "0 0 0", 'range': "-0.00167821 0.0335354"},
                            {'axis': "-0.124293 0.989762 0.0701648", 'name': "knee_angle_l_rotation3", 'pos': "0 0 0", 'range': "-0.262788 -1.08939e-08"}
                        ],
                        'geoms': [
                            {'mesh': "l_tibia", 'name': "l_tibia", 'type': "mesh"},
                            {'mesh': "l_fibula", 'name': "l_fibula", 'type': "mesh"},
                            {'name': "GasLat_at_shank_l_wrap", 'pos': "-0.0074 -0.074 0.0033", 'quat': "-0.0298211 -0.737282 -0.655511 -0.160722", 'class': "wrap", 'size': "0.055 0.05"},
                            {'name': "GasMed_at_shank_l_wrap", 'pos': "-0.0074 -0.074 0.0033", 'quat': "0.073733 -0.735403 -0.67187 -0.048347", 'class': "wrap", 'size': "0.055 0.05"},
                            {'name': "GR_at_condyles_l_wrap", 'pos': "-0.003 -0.02 0", 'quat': "0.980067 0 0.198669 0", 'class': "wrap", 'size': "0.036 0.05"},
                            {'name': "SM_at_condyles_l_wrap", 'pos': "-0.001 -0.02 0", 'quat': "0.99875 0 0.0499792 0", 'class': "wrap", 'size': "0.0352 0.05"},
                            {'name': "ST_at_condyles_l_wrap", 'pos': "-0.002 -0.0205 0", 'quat': "0.995004 0 0.0998334 0", 'class': "wrap", 'size': "0.0425 0.05"},
                            {'name': "BF_at_gastroc_l_wrap", 'pos': "-0.058 -0.06 0", 'class': "wrap", 'size': "0.03 0.075"}
                        ],
                        'sites': [
                            {'name': "knee_l_location", 'pos': "0 0 0"}, 
                            {'name': "knee_l", 'class': "myo_leg_marker"},
                            {'name': "bflh_l-P2", 'pos': "-0.0337 -0.035 -0.0253"},
                            {'name': "bflh_l-P3", 'pos': "-0.0287 -0.0455 -0.0303"},
                            {'name': "bfsh_l-P2", 'pos': "-0.0301 -0.0419 -0.0318"},
                            {'name': "edl_l-P1", 'pos': "-0.016 -0.1157 -0.0205"},
                            {'name': "edl_l-P2", 'pos': "0.0164 -0.376 -0.0112"},
                            {'name': "ehl_l-P1", 'pos': "-0.014 -0.155 -0.0189"},
                            {'name': "ehl_l-P2", 'pos': "0.0071 -0.2909 -0.0164"},
                            {'name': "ehl_l-P3", 'pos': "0.02 -0.3693 0.0028"},
                            {'name': "fdl_l-P1", 'pos': "-0.0023 -0.1832 0.0018"},
                            {'name': "fdl_l-P2", 'pos': "-0.0176 -0.3645 0.0124"},
                            {'name': "fhl_l-P1", 'pos': "-0.031 -0.2163 -0.02"},
                            {'name': "fhl_l-P2", 'pos': "-0.0242 -0.3671 0.0076"},
                            {'name': "grac_l-P2", 'pos': "-0.0184 -0.0476 0.0296"},
                            {'name': "grac_l-P3", 'pos': "0.0018 -0.0696 0.0157"},
                            {'name': "perbrev_l-P1", 'pos': "-0.0243 -0.2532 -0.0251"},
                            {'name': "perbrev_l-P2", 'pos': "-0.0339 -0.3893 -0.0249"},
                            {'name': "perbrev_l-P3", 'pos': "-0.0285 -0.4004 -0.0255"},
                            {'name': "perlong_l-P1", 'pos': "-0.02 -0.1373 -0.0282"},
                            {'name': "perlong_l-P2", 'pos': "-0.0317 -0.39 -0.0237"},
                            {'name': "perlong_l-P3", 'pos': "-0.0272 -0.4014 -0.024"},
                            {'name': "recfem_l-P5", 'pos': "0.0326 -0.0631 0.0005"},
                            {'name': "sart_l-P3", 'pos': "-0.0251 -0.0401 0.0365"},
                            {'name': "sart_l-P4", 'pos': "-0.0159 -0.0599 0.0264"},
                            {'name': "sart_l-P5", 'pos': "0.0136 -0.081 0.0026"},
                            {'name': "semimem_l-P2", 'pos': "-0.029 -0.0417 0.0196"},
                            {'name': "semiten_l-P2", 'pos': "-0.0312 -0.0508 0.0229"},
                            {'name': "semiten_l-P3", 'pos': "0.0019 -0.0773 0.0117"},
                            {'name': "soleus_l-P1", 'pos': "-0.0076 -0.0916 -0.0098"},
                            {'name': "tfl_l-P4", 'pos': "0.0108 -0.041 -0.0346"},
                            {'name': "tibant_l-P1", 'pos': "0.0154 -0.1312 -0.0162"},
                            {'name': "tibant_l-P2", 'pos': "0.0251 -0.1906 -0.0128"},
                            {'name': "tibant_l-P3", 'pos': "0.0233 -0.3659 0.0132"},
                            {'name': "tibpost_l-P1", 'pos': "-0.0041 -0.1304 -0.0103"},
                            {'name': "tibpost_l-P2", 'pos': "-0.0164 -0.3655 0.0175"},
                            {'name': "vasint_l-P5", 'pos': "0.0326 -0.0632 -0.0004"},
                            {'name': "vaslat_l-P5", 'pos': "0.0325 -0.0634 -0.0051"},
                            {'name': "vasmed_l-P5", 'pos': "0.0319 -0.0636 0.0068"},
                            {'name': "LKJC", 'pos': "0.0017 -0.0024 0.0085"},
                            {'name': "LTB1", 'pos': "-0.0017 -0.1565 -0.0492"},
                            {'name': "LTB2", 'pos': "0.037 -0.2301 0.0039"},
                            {'name': "LTB3", 'pos': "0.0114 -0.2952 -0.0554"},
                            {'name': "LLMAL", 'pos': "-0.005 -0.3888 -0.053"},
                            {'name': "LMMAL", 'pos': "0.006 -0.3888 0.038"},
                            {'name': "L_tibial_plateau", 'pos': "-0.0081 -0.017 0.0015"},
                            {'name': "GasLat_at_shank_l_site_gaslat_l_side", 'pos': "-0.0512647 -0.0235815 0.0514609"},
                            {'name': "GasMed_at_shank_l_site_gasmed_l_side", 'pos': "-0.0226964 -0.1301 -0.0498879"},
                            {'name': "GR_at_condyles_l_site_grac_l_side", 'pos': "-0.0513804 -0.042587 -0.0198669"},
                            {'name': "SM_at_condyles_l_site_semimem_l_side", 'pos': "-0.0387876 -0.035171 0.0182577"},
                            {'name': "ST_at_condyles_l_site_semiten_l_side", 'pos': "-0.0428471 -0.0468901 0.0181631"},
                            {'name': "BF_at_gastroc_l_site_bfsh_l_side", 'pos': "-0.0687049 -0.0256284 -0.0426731"}
                        ],
                        'bodies': [
                            {
                                'name': "talus_l",
                                'pos': " ".join(map(str,Muscletalus_l_pos)),
                                'joints': [{'axis': "0.105014 0.174022 0.979126", 'name': "ankle_angle_l", 'pos': "0 0 0", 'range': "-0.698132 0.523599"}],
                                'geoms': [{'mesh': "l_talus", 'name': "l_talus", 'type': "mesh"}],
                                'sites': [
                                    {'name': "LAJC", 'pos': "0 0 0"},
                                    {'name': "ankle_l_location", 'pos': "0 0 0"}, 
                                    {'name': "ankle_l", 'class': "myo_leg_marker"}
                                    ],
                                'bodies': [
                                    {
                                        'name': "calcn_l",
                                        'pos': " ".join(map(str,Musclecalcn_l_pos)),
                                        'inertials': [{'pos': "0.0821377 0.0108024 0.000944392", 'quat': "0.541341 0.502987 0.458598 0.493601", 'mass': " ".join(map(str,Musclecalcn_l_mass)), 'diaginertia': "0.00313636 0.00297113 0.000941737"}],
                                        'joints': [{'axis': "-0.78718 -0.604747 -0.120949", 'name': "subtalar_angle_l", 'pos': "0 0 0", 'range': "-0.349066 0.349066"}],
                                        'geoms': [{'mesh': "l_foot", 'name': "l_foot", 'type': "mesh"}],
                                        'sites': [
                                            {'name': "edl_l-P3", 'pos': "0.0919 0.036 -0.0008"},
                                            {'name': "edl_l-P4", 'pos': "0.1616 0.0055 -0.013"},
                                            {'name': "ehl_l-P4", 'pos': "0.097 0.0389 0.0211"},
                                            {'name': "ehl_l-P5", 'pos': "0.1293 0.0309 0.0257"},
                                            {'name': "ehl_l-P6", 'pos': "0.1734 0.0139 0.028"},
                                            {'name': "fdl_l-P3", 'pos': "0.0436 0.0315 0.028"},
                                            {'name': "fdl_l-P4", 'pos': "0.0708 0.0176 0.0263"},
                                            {'name': "fdl_l-P5", 'pos': "0.1658 -0.0081 -0.0116"},
                                            {'name': "fhl_l-P3", 'pos': "0.0374 0.0276 0.0241"},
                                            {'name': "fhl_l-P4", 'pos': "0.1038 0.0068 0.0256"},
                                            {'name': "fhl_l-P5", 'pos': "0.1726 -0.0053 0.0269"},
                                            {'name': "gaslat_l-P2", 'pos': "0.0044 0.031 0.0053"},
                                            {'name': "gasmed_l-P2", 'pos': "0.0044 0.031 0.0053"},
                                            {'name': "perbrev_l-P4", 'pos': "0.0471 0.027 -0.0233"},
                                            {'name': "perbrev_l-P5", 'pos': "0.0677 0.0219 -0.0343"},
                                            {'name': "perlong_l-P4", 'pos': "0.0438 0.023 -0.0221"},
                                            {'name': "perlong_l-P5", 'pos': "0.0681 0.0106 -0.0284"},
                                            {'name': "perlong_l-P6", 'pos': "0.0852 0.0069 -0.0118"},
                                            {'name': "perlong_l-P7", 'pos': "0.1203 0.0085 0.0184"},
                                            {'name': "soleus_l-P2", 'pos': "0.0044 0.031 0.0053"},
                                            {'name': "tibant_l-P4", 'pos': "0.1166 0.0178 0.0305"},
                                            {'name': "tibpost_l-P3", 'pos': "0.0417 0.0334 0.0286"},
                                            {'name': "tibpost_l-P4", 'pos': "0.0772 0.0159 0.0281"},
                                            {'name': "LCAL", 'pos': "-0.025 0.02 0.005"},
                                            {'name': "LTOE", 'pos': "0.205 0.0297 0.03"},
                                            {'name': "LMT5", 'pos': "0.145 0.0249 -0.059"},
                                            {'name': "l_foot_touch", 'type': "box", 'pos': "0.09 -.01 0.0", 'size': ".1 .01 .055", 'euler': "0 0 0", 'class': "myo_leg_touch"}
                                        ],
                                        'bodies': [
                                            {
                                                'name': "toes_l",
                                                'pos': " ".join(map(str,Muscletoes_l_pos)),
                                                'joints': [{'axis': "-0.580954 0 -0.813936", 'name': "mtp_angle_l", 'pos': "0 0 0", 'range': "-0.523599 0.523599"}],
                                                'geoms': [{'mesh': "l_bofoot", 'name': "l_bofoot", 'type': "mesh"}],
                                                'sites': [
                                                    {'name': "toe_l_location", 'pos': "0 0 0"}, 
                                                    {'name': "toe_l", 'class': "myo_leg_marker"},
                                                    {'name': "l_toes_touch", 'type': "box", 'pos': "0.0275 -.01 -.000", 'size': ".04 .01 .0675", 'euler': "0 .7 0", 'class': "myo_leg_touch"},
                                                    {'name': "edl_l-P5", 'pos': "0.0003 0.0047 -0.0153"},
                                                    {'name': "edl_l-P6", 'pos': "0.0443 -0.0004 -0.025"},
                                                    {'name': "ehl_l-P7", 'pos': "0.0298 0.0041 0.0245"},
                                                    {'name': "ehl_l-P8", 'pos': "0.0563 0.0034 0.0186"},
                                                    {'name': "fdl_l-P6", 'pos': "-0.0019 -0.0078 -0.0147"},
                                                    {'name': "fdl_l-P7", 'pos': "0.0285 -0.0071 -0.0215"},
                                                    {'name': "fdl_l-P8", 'pos': "0.0441 -0.006 -0.0242"},
                                                    {'name': "fhl_l-P6", 'pos': "0.0155 -0.0064 0.0265"},
                                                    {'name': "fhl_l-P7", 'pos': "0.0562 -0.0102 0.0181"}
                                                ]
                                            }
                                        ]
                                    }
                                ]
                            }
                        ]
                    },
                    {
                        'name': "patella_l",
                        'pos': " ".join(map(str,Musclepatella_l_pos)),
                        'joints': [
                            {'axis': "0 1 0", 'name': "knee_angle_l_beta_translation2", 'pos': "0 0 0", 'range': "-0.0408267 -0.0108281", 'type': "slide"},
                            {'axis': "1 0 0", 'name': "knee_angle_l_beta_translation1", 'pos': "0 0 0", 'range': "-0.0227731 0.0524192", 'type': "slide"},
                            {'axis': "0 0 1", 'name': "knee_angle_l_beta_rotation1", 'pos': "0 0 0", 'range': "-1.79241 0.010506"}
                        ],
                        'geoms': [{'mesh': "l_patella", 'name': "l_patella", 'type': "mesh"}],
                        'sites': [
                            {'name': "recfem_l-P2", 'pos': "0.01 0.049 -0.0007"},
                            {'name': "recfem_l-P3", 'pos': "0.0121 0.0437 0.001"},
                            {'name': "recfem_l-P4", 'pos': "0.005 0.0025 0"},
                            {'name': "vasint_l-P3", 'pos': "0.0058 0.048 0.0006"},
                            {'name': "vasint_l-P4", 'pos': "0.005 0.0025 0.0004"},
                            {'name': "vaslat_l-P3", 'pos': "0.0103 0.0423 -0.0141"},
                            {'name': "vaslat_l-P4", 'pos': "0.005 0.0025 -0.0073"},
                            {'name': "vasmed_l-P3", 'pos': "0.0063 0.0445 0.017"},
                            {'name': "vasmed_l-P4", 'pos': "0.005 0.0025 0.0085"}
                        ]
                    }
                ]
            }
        ]
    }
  ]

# Add all bodies to the worldbody
for body in bodies_data:
    add_body(worldbody_node, body)



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
