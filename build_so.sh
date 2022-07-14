#!/bin/bash

clear
g++ Src/*/*.cpp -m64 -std=c++14 -O2 -c -fopenmp -fPIC -Wno-invalid-offsetof
g++ *.o -O2 -shared -fopenmp -fPIC -o libRiemann.so
rm *.o
mv libRiemann.so Test/
echo "build libRiemann.so done"