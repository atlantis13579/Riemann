#!/bin/bash

g++ *.cpp ../Src/*/*.cpp -m64 -std=c++14 -O2 -fPIC -Wno-invalid-offsetof -o test