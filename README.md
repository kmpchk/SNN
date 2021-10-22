# Emotico 2.0
Spiking Neural Network with Optimized Spiking Neuron Model.

## Prerequisites
   * GCC 7 and higher
   * cmake 3.8 and higher

## Getting Started
Clone repo:
```sh
git clone --recurse-submodules https://github.com/kmpchk/SNN.git
```
Build:
```sh
mkdir build_release && cd build_release
cmake -DCMAKE_BUILD_TYPE=Release
make -j$(nproc --all)
```