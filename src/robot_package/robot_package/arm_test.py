import sys
import numpy as np
from typing import NamedTuple
import pickle

class CoordPixel(NamedTuple):
    """Pixel coordinate (u: left->right, v: top->bottom)"""
    u: int  # pixel
    v: int  # pixel

class Coord3D(NamedTuple):
    """3D coordinate (x: left->right, y: bottom->top, z: far->near)"""
    x: float  # mm
    y: float  # mm
    z: float  # mm

class CoordRobot(NamedTuple):
    """Operational coordinate of robot (angles of the motors)
    theta_1: zero at middle, counter-clockwise is positive direction
    theta_2: zero at vertical, spin from front to back is positive direction
    theta_3: zero when the two arms are aligned, spin from back to front is positive direction"""
    theta_1: float  # degree
    theta_2: float  # degree
    theta_3: float  # degree

class CamCoordTransformer:
    def __init__(self):
        self.cam_offset = Coord3D(0, 140, 120) # y, x, z
        self.cam_angle = 180
        self.intrinsic_mat_file = None
        self.pixel_width = 640 #2592,
        self.pixel_height = 480 #1944,

        self.intrinsic_mat = self.load_intrinsic_mat(self.intrinsic_mat_file, self.pixel_width, self.pixel_height)
        self.rotation_mat = self.get_rotation_mat(self.cam_angle)

    def load_intrinsic_mat(self, mat_file: str, pixel_width: int, pixel_height: int):
        """mat_file: path to pickled intrinsic matrix file (if None, use f=2500 based on measured intrinsic matrix)"""
        mat = None
        if mat_file is None:
            cam_f = 2500.0
            fx = cam_f / 2592.0 * pixel_width
            fy = cam_f / 1944.0 * pixel_height
            mat = np.array([
                [  fx, 0.0, 0.0],
                [ 0.0,  fy, 0.0],
                [ 0.0, 0.0, 1.0],
            ])
        else:
            with open(mat_file, "rb") as f:
                mat = np.array(pickle.load(f))

		# cx, cy assigned with actual center of the image
        mat[0, 2] = (pixel_width - 1) / 2.0
        mat[1, 2] = (pixel_height - 1) / 2.0
        return mat
        
    def get_rotation_mat(self, cam_angle: float):
        """camera_angle: degree compared to vertical axis (default to 20 degree based on measured extrinsic matrix)"""
        s = np.sin(np.deg2rad(cam_angle))
        c = np.cos(np.deg2rad(cam_angle))
        mat = np.array([   #TODO verifyy
            [1.0, 0.0, 0.0],
            [0.0,  -c,  -s],
            [0.0,   s,  -c],
        ])
        return mat

    def pixel_to_world_coord(self, pixel_coord: CoordPixel, target_z: float) -> Coord3D:
        """convert pixel coord to world 3D coord (target depth must be given)"""
        target_z = np.absolute(target_z)
        img_coord = np.array([pixel_coord.u, pixel_coord.v, 1.0])
        cam_coord = np.matmul(np.linalg.inv(self.intrinsic_mat), img_coord)
        cam_coord = cam_coord / cam_coord[2] * target_z
        world_coord = np.matmul(np.linalg.inv(self.rotation_mat), cam_coord)
        world_coord = world_coord / world_coord[2] * target_z * -1.0
        x = world_coord[0] + self.cam_offset.x
        y = world_coord[1] + self.cam_offset.y
        z = world_coord[2] + self.cam_offset.z
        return Coord3D(x, y, z)

if __name__ == '__main__':
    arm = CamCoordTransformer()

    pixel_coord = CoordPixel(320, 240)
    target_z = 120.0 #  mm
    world_coord = arm.pixel_to_world_coord(pixel_coord, target_z)

    print(world_coord)