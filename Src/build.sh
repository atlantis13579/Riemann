#!/bin/bash

clear
g++ Collision/*.cpp -std=c++11 -O2 -c -fPIC
g++ CollisionPrimitive/AxisAlignedBox3d.cpp -std=c++11 -O2 -c -fPIC
g++ CollisionPrimitive/Capsule3d.cpp -std=c++11 -O2 -c -fPIC
g++ CollisionPrimitive/HeightField3d.cpp -std=c++11 -O2 -c -fPIC
g++ CollisionPrimitive/MeshBVH4.cpp -std=c++11 -O2 -c -fPIC
g++ CollisionPrimitive/OrientedBox3d.cpp -std=c++11 -O2 -c -fPIC
g++ CollisionPrimitive/Segment3d.cpp -std=c++11 -O2 -c -fPIC
# g++ CollisionPrimitive/TriangleMesh.cpp -std=c++11 -O2 -c -fPIC
g++ ImageSpace/ImageProcessing.cpp -m64 -std=c++11 -O2 -c -fPIC
g++ LinearSystem/*.cpp -m64 -std=c++11 -O2 -c -fPIC
g++ Maths/*.cpp -m64 -std=c++11 -O2 -c -fPIC
g++ Optimization/*.cpp -m64 -std=c++11 -O2 -c -fPIC
g++ PositionBasedDynamics/*.cpp -m64 -std=c++11 -O2 -c -fPIC
g++ RigidBodyDynamics/*.cpp -m64 -std=c++11 -O2 -c -fPIC
g++ Tools/*.cpp -m64 -std=c++11 -O2 -c -fPIC

g++ *.o -O2 -shared -fPIC -o libRiemann.so
rm *.o