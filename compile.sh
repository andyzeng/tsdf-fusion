#!/bin/bash

export PATH=$PATH:/usr/local/cuda/bin

if uname | grep -q Darwin; then
  CUDA_LIB_DIR=/usr/local/cuda/lib
elif uname | grep -q Linux; then
  CUDA_LIB_DIR=/usr/local/cuda/lib64
fi

nvcc -std=c++11 -O3 -o demo demo.cu -I/usr/local/cuda/include -L$CUDA_LIB_DIR -lcudart -lcublas -lcurand -D_MWAITXINTRIN_H_INCLUDED `pkg-config --cflags --libs opencv`









