#!/usr/bin/env bash
cd build
cmake ..
make
cd ..
python detect_SIFT.py
./build/do_3D