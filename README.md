# Model Predictive Control

This project uses Model Predictive Control (MPC) to solve control problems.

## Requirements

The requirements are:

- CMake 3.11 or better; 3.14+ highly recommended
- A C++17 compatible compiler
- The Boost libararies
- Git
- Doxygen (optional, highly recommended)

## Instructions

To configure (Debug):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
```

To configure (Release):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```

Add `-GNinja` if you have Ninja.

To build:

```bash
cmake --build build
```

To test (`--target` can be written as `-t` in CMake 3.15+):

```bash
cmake --build build --target test
```

To test by tag
```
cd build && ctest -R <tag>
```

To run
```
./build/apps/app
```

To build and run
```
cmake --build build && ./build/apps/app
```

To build and test
```
cmake --build build && cmake --build build --target test
```

To build docs (requires Doxygen, output in `build/docs/html`):

```bash
cmake --build build --target docs
```
