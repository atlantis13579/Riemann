#!/bin/bash

g++ CollisionPrimitive/AxisAlignedBox3d.cpp -std=c++11 -O2 -c -fPIC
g++ CollisionPrimitive/Capsule3d.cpp -std=c++11 -O2 -c -fPIC
g++ CollisionPrimitive/HeightField3d.cpp -std=c++11 -O2 -c -fPIC
g++ CollisionPrimitive/MeshBVH4.cpp -std=c++11 -O2 -c -fPIC
g++ CollisionPrimitive/OrientedBox3d.cpp -std=c++11 -O2 -c -fPIC
g++ CollisionPrimitive/Segment3d.cpp -std=c++11 -O2 -c -fPIC
# g++ CollisionPrimitive/TriangleMesh.cpp -std=c++11 -O2 -c -fPIC
g++ ImageSpace/ImageProcessing.cpp -std=c++11 -O2 -c -fPIC
g++ LinearSystem/PolarDecomposition.cpp -std=c++11 -O2 -c -fPIC
g++ LinearSystem/SVDDecomposition.cpp -std=c++11 -O2 -c -fPIC
g++ Maths/Bezier.cpp -std=c++11 -O2 -c -fPIC
g++ Maths/Matrix3d.cpp -std=c++11 -O2 -c -fPIC

g++ *.o -O2 -shared -fPIC -o libRiemann.so
rm *.o