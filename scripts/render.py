import time
import trimesh
import trimesh.creation
import pyrender
import numpy as np
import sys


class Excavator:
    @staticmethod
    def _load_mesh(path):
        mesh = trimesh.load(path, force='mesh')
        mesh = pyrender.Mesh.from_trimesh(mesh)
        return mesh

    def __init__(self, path):
        self.path = path
        scene = pyrender.Scene()

        scene.add(Excavator._load_mesh('/home/yorick/Downloads/volvo_undercarage.obj/617afeb7-bd47-42cd-880a-2037f2365277.obj'), name='undercarage')

        scene.add(
            Excavator._load_mesh('/home/yorick/Downloads/volo_frame.obj/63c51b54-ee9b-4f30-8d95-73b928d979de.obj'),
            name='frame',
            parent_name='undercarage',
            pose=translation_matrix(0.0, 0.0, 1.0))

        pose = translation_matrix(0.16, 0.0, 0.0)
        pose = np.dot(pose, rotation_matrix(0.0, 0.0))

        scene.add(Excavator._load_mesh('/home/yorick/Downloads/volvo_boom.obj/a1dae9a0-722a-4ff0-a658-c6082e90bc6f.obj'), name='boom', parent_name='frame', pose=pose)

        scene.add(
            Excavator._load_mesh('/home/yorick/Downloads/volvo_arm.obj/be8acb75-fa50-4c48-850e-59a523e38074.obj'),
            name='arm',
            parent_name='boom',
            pose=translation_matrix(-0.25, 0.0, 7.84))

        boxf_mesh = pyrender.Mesh.from_trimesh(trimesh.creation.box(extents=[10, 10, 0]), smooth=False)
        scene.add(boxf_mesh, name='floor')

        cam = pyrender.PerspectiveCamera(yfov=(np.pi / 3.0))
        cam_pose = np.array([
            [0.0,  -np.sqrt(2)/2, np.sqrt(2)/2, 5.5],
            [1.0, 0.0,           0.0,           0.0],
            [0.0,  np.sqrt(2)/2,  np.sqrt(2)/2, 5.4],
            [0.0,  0.0,           0.0,          1.0]
        ])

        scene.add(cam, name='camera', pose=cam_pose)

        for node in scene.meshes:
            node.primitives[0].material.alphaMode = 'OPAQUE'
            node.primitives[0].material.roughnessFactor = 1.0
            node.primitives[0].material.metallicFactor = 1.0

        self.scene = scene

    def render(self):
        self._viewer = pyrender.Viewer(self.scene, use_raymond_lighting=True, run_in_thread=True)


def _ry_matrix(theta):
    """Rotation matrix around the Y axis"""
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])


def _rz_matrix(theta):
    """Rotation matrix around the Z axis"""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])


def translation_matrix(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


def rotation_matrix(pitch, yaw):
    cartesian_matrix = np.dot(_rz_matrix(yaw), _ry_matrix(pitch))

    homogeneous_matrix = np.identity(4)
    homogeneous_matrix[: -1, : -1] = cartesian_matrix

    return homogeneous_matrix


scene = pyrender.Scene()
scene.add(load_mesh('/home/yorick/Downloads/volvo_undercarage.obj/617afeb7-bd47-42cd-880a-2037f2365277.obj'), name='undercarage')

scene.add(
    load_mesh('/home/yorick/Downloads/volo_frame.obj/63c51b54-ee9b-4f30-8d95-73b928d979de.obj'),
    name='frame',
    parent_name='undercarage',
    pose=translation_matrix(0.0, 0.0, 1.0))

pose = translation_matrix(0.16, 0.0, 0.0)
pose = np.dot(pose, rotation_matrix(0.0, 0.0))


scene.add(load_mesh('/home/yorick/Downloads/volvo_boom.obj/a1dae9a0-722a-4ff0-a658-c6082e90bc6f.obj'), name='boom', parent_name='frame', pose=pose)
scene.add(load_mesh('/home/yorick/Downloads/volvo_arm.obj/be8acb75-fa50-4c48-850e-59a523e38074.obj'), name='arm', parent_name='boom', pose=np.array([
    [1.0, 0.0, 0.0, -0.25],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 7.84],
    [0.0, 0.0, 0.0, 1.0]
]))


boxf_mesh = pyrender.Mesh.from_trimesh(trimesh.creation.box(extents=[10, 10, 0]), smooth=False)
scene.add(boxf_mesh, name='floor')


cam = pyrender.PerspectiveCamera(yfov=(np.pi / 3.0))
cam_pose = np.array([
    [0.0,  -np.sqrt(2)/2, np.sqrt(2)/2, 5.5],
    [1.0, 0.0,           0.0,           0.0],
    [0.0,  np.sqrt(2)/2,  np.sqrt(2)/2, 5.4],
    [0.0,  0.0,           0.0,          1.0]
])

cam_node = scene.add(cam, name='camera', pose=cam_pose)


for node in scene.meshes:
    node.primitives[0].material.alphaMode = 'OPAQUE'
    node.primitives[0].material.roughnessFactor = 1.0
    node.primitives[0].material.metallicFactor = 1.0

v = pyrender.Viewer(scene, use_raymond_lighting=True, run_in_thread=True)


frame_node = list(scene.get_nodes(name='frame'))[0]
frame_pose_init = scene.get_pose(frame_node)

boom_node = list(scene.get_nodes(name='boom'))[0]
boom_pose_init = scene.get_pose(boom_node)


i = 0
while v.is_active:
    v.render_lock.acquire()

    boom_pose = np.dot(boom_pose_init, rotation_matrix(np.sin(i)+1.3, 0))
    scene.set_pose(boom_node, boom_pose)

    frame_pose = np.dot(frame_pose_init, rotation_matrix(0.0, np.sin(i*0.5)*2))
    scene.set_pose(frame_node, frame_pose)

    v.render_lock.release()
    i += 0.01
    time.sleep(0.01)
