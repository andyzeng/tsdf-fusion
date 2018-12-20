# Volumetric TSDF Fusion of Multiple Depth Maps

**Update**: a python version of this code with both CPU/GPU support can be found [here](https://github.com/andyzeng/tsdf-fusion-python).

![Teaser](teaser.jpg?raw=true)

CUDA/C++ code to fuse multiple registered depth maps into a projective truncated signed distance function (TSDF) voxel volume, which can then be used to create high quality 3D surface meshes and point clouds. Tested on Ubuntu 14.04 and 16.04.

Looking for an older version? See [here](old-version).

## Change Log
* **Nov. 1, 2017.** Bug fix: `tsdf2mesh.m` now properly generates a mesh in camera coordinates instead of voxel coordinates.
* **Oct. 30, 2017.** Notice: changed default weight threshold for `SaveVoxelGrid2SurfacePointCloud` in demo code to enable creating point cloud visualizations with only one depth frame.
* **Aug. 30, 2017.** Bug fix: remove deprecated offsets from surface distance compute during integration.

## Requirements
 * NVIDA GPU with [CUDA](https://developer.nvidia.com/cuda-downloads) support
 * [OpenCV](http://opencv.org/) (tested with OpenCV 2.4.11)

## Demo
This demo fuses 50 registered depth maps from directory `data/rgbd-frames` into a projective TSDF voxel volume, and creates a 3D surface point cloud `tsdf.ply`, which can be visualized with a 3D viewer like [Meshlab](http://www.meshlab.net/).

**Note**: Input depth maps should be saved in format: 16-bit PNG, depth in millimeters.

```shell
./compile.sh # compiles demo executable
./demo # 3D point cloud saved to tsdf.ply and voxel grid saved to tsdf.bin
```

[Optional] This demo also saves the computed voxel volume into a binary file `tsdf.bin`. Run the following script in Matlab to create a 3D surface mesh `mesh.ply`, which can be visualized with [Meshlab](http://www.meshlab.net/).

```matlab
tsdf2mesh; % 3D mesh saved to mesh.ply
```

## Seen in
 * [3DMatch: Learning Local Geometric Descriptors from RGB-D Reconstructions (CVPR 2017)](http://3dmatch.cs.princeton.edu/)
 * [Semantic Scene Completion from a Single Depth Image (CVPR 2017)](http://sscnet.cs.princeton.edu/)
 * [Deep Sliding Shapes for Amodal 3D Object Detection in RGB-D Images (CVPR 2016)](http://dss.cs.princeton.edu/)

## References
 * [A Volumetric Method for Building Complex Models from Range Images (SIGGRAPH 1996)](https://graphics.stanford.edu/papers/volrange/volrange.pdf)
 * [KinectFusion: Real-Time Dense Surface Mapping and Tracking (ISMAR 2011)](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf)
 * [Scene Coordinate Regression Forests for Camera Relocalization in RGB-D Images (CVPR 2013)](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/RelocForests.pdf)

### Citing

This repository is a part of [3DMatch Toolbox](https://github.com/andyzeng/3dmatch-toolbox). If you find this code useful in your work, please consider citing:

```
@inproceedings{zeng20163dmatch, 
    title={3DMatch: Learning Local Geometric Descriptors from RGB-D Reconstructions}, 
    author={Zeng, Andy and Song, Shuran and Nie{\ss}ner, Matthias and Fisher, Matthew and Xiao, Jianxiong and Funkhouser, Thomas}, 
    booktitle={CVPR}, 
    year={2017} 
}
```