import ctypes
import os
from xml.sax.handler import property_interning_dict
import numpy as np

class PhysxHelper(object):
    def __init__(self):
        path = os.path.dirname(os.path.abspath(__file__))
        sopath = os.path.join(path, "../Src/libRiemann.so")
        print ("load so : ", sopath)

        libs = ctypes.cdll.LoadLibrary(sopath)
        # print(libs)

        libs.LoadPhysxScene.restype = ctypes.c_void_p
        libs.RayCast.restype = ctypes.c_float
        libs.RayCast2.restype = ctypes.c_float

        self._libs = libs
        self._scene = None
        self._image = None

    def __del__(self):
        if self._scene:
            self._libs.DeletePhysxScene(ctypes.c_void_p(self._scene))
            self._scene = None

    def load_scene(self, file):
        path = os.path.dirname(os.path.abspath(__file__))
        datapath = os.path.join(path, file)
        datapath = bytes(datapath, encoding='utf8')
        scene = self._libs.LoadPhysxScene(ctypes.c_char_p(datapath))
        if not scene:
            print ("load data failed : " + datapath)
            return
        print("load space : %s" % datapath)
        self._scene = scene

    def raycast(self, pos, dir):
        if not self._scene:
            print ("scene not load")
            return None
        c_float = ctypes.c_float
        f = self._libs.RayCast(ctypes.c_void_p(self._scene), \
                               c_float(pos[0]), c_float(pos[1]), c_float(pos[2]), \
                               c_float(dir[0]), c_float(dir[1]), c_float(dir[2]))
        if f < 0.0 or f > 10000000.0:
            print ("not hit")
            return None
        return (pos[0] + f * dir[0], pos[1] + f * dir[1], pos[2] + f * dir[2])

    def create_image(self, width, height):
        arr = np.array([0] * width * height, dtype=np.float32)
        # print(arr.shape)
        self._width = width
        self._height = height
        self._image = arr

    def render_depth_image(self, pos, yaw, fov = 1.0):
        if (self._image is not None) and (self._scene is not None):
            dataptr = self._image.ctypes.data_as(ctypes.c_void_p)
            # print(dataptr)
            c_float = ctypes.c_float
            c_int = ctypes.c_int
            debug_output = False
            dir_x = math.sin(yaw)
            dir_z = math.cos(yaw)
            self._libs.RenderDepthImage(ctypes.c_void_p(self._scene), dataptr, \
                               c_int(self._width), c_int(self._height), c_float(fov), c_float(0.1), c_float(100.0), \
                               c_float(pos[0]), c_float(pos[1]), c_float(pos[2]), \
                               c_float(dir_x), c_float(0.0), c_float(dir_z), 
                               c_float(0.0), c_float(1.0), c_float(0.0), debug_output)
        
print("test")

physx = PhysxHelper()
physx.load_scene("../TestData/Japan.xml.bin")
hitpos = physx.raycast((-521.23, 55.87, 399.15), (0, -1, 0))
print(hitpos)
physx.create_image(200, 200)

pos = (1768.8, 19.5, 1551.5)
yaw = -0.27
physx.render_depth_image((pos[0], pos[1] + 1.8, pos[2]), yaw)