#!/bin/bash

clear
g++ */*.cpp -m64 -std=c++14 -O2 -c -fPIC
g++ *.o -O2 -shared -fPIC -o libRiemann.so
rm *.o