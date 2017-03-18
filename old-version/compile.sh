export PATH=$PATH:/usr/local/cuda/bin
CUDA_LIB_DIR=/usr/local/cuda/lib64
CUDNN_LIB_DIR=/usr/local/cudnn/v4rc/lib64

nvcc kinfu.cu -std=c++11 -Iinclude -lm -lpng `libpng-config --cflags` -o kinfu