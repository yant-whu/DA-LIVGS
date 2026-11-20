# Environment

Validated environment: Ubuntu 20.04, ROS Noetic, GCC with C++17, CUDA 11.7, cuDNN 8.x, LibTorch 2.0.1 CUDA 11.7, TensorRT 8.6.x, OpenCV 4.x, PCL 1.13.x, Ceres 2.x with SuiteSparse, Eigen 3.3+, yaml-cpp 0.5+, Python 3, NumPy, and an NVIDIA GPU.

CMake accepts `OpenCV_DIR`, `Torch_DIR`, `CUDA_HOME`, `TENSORRT_ROOT`, and `GLM_INCLUDE_DIR`. The validated build uses OpenCV 4.7 with PCL 1.13.1. TensorRT engines are generated locally.

For local Ceres and Livox SDK installations, set `Ceres_DIR` and `LIVOX_SDK_ROOT`, then pass them through `DA_LIVGS_CMAKE_ARGS`.

```bash
export DA_LIVGS_DEPENDENCY_ROOT=/home/yt/Libraries
export DA_LIVGS_CMAKE_ARGS="-DCeres_DIR=$DA_LIVGS_DEPENDENCY_ROOT/ceres-2.0.0/install/lib/cmake/Ceres -DLIVOX_SDK_ROOT=$DA_LIVGS_DEPENDENCY_ROOT/livox-sdk-2.3.0/install"
```

The validated build used PCL 1.13.1. With the local ROS, CUDA, LibTorch, TensorRT, Ceres, Livox SDK, PCL, Eigen, and GLM installations exposed through the documented variables, both production targets completed compilation and linking. The compiler emitted upstream deprecation warnings from TensorRT, LibTorch, and CUDA headers. No build or devel artifacts are retained in this repository.
