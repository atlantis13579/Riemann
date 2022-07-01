import ctypes
import os

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
        self._scene = {}

    def load_scene(self, spaceno, file):
        path = os.path.dirname(os.path.abspath(__file__))
        datapath = os.path.join(path, file)
        datapath = bytes(datapath, encoding='utf8')
        scene = self._libs.LoadPhysxScene(ctypes.c_char_p(datapath))
        if not scene:
            print ("load data failed : " + datapath)
            return
        self._scene[spaceno] = scene

    def delete_scene(self, spaceno):
        if spaceno in self.scenes:
            self._physx.delete_scene(ctypes.c_void_p(self.scenes[spaceno]))
            del self.scenes[spaceno]

    def raycast(self, spaceno, pos, dir):
        scene = self._scene.get(spaceno, None)
        if not scene:
            print ("space not load : ", spaceno)
            return None
        c_float = ctypes.c_float
        f = self._libs.RayCast(ctypes.c_void_p(scene), \
                               c_float(pos[0]), c_float(pos[1]), c_float(pos[2]), \
                               c_float(dir[0]), c_float(dir[1]), c_float(dir[2]))
        if f < 0.0 or f > 10000000.0:
            print ("not hit")
            return None
        return (pos[0] + f * dir[0], pos[1] + f * dir[1], pos[2] + f * dir[2])

print("test")

physx = PhysxHelper()
physx.load_scene(105, "data/Japan.xml.bin")
hitpos = physx.raycast(105, (-521.23, 55.87, 399.15), (0, -1, 0))
print(hitpos)
