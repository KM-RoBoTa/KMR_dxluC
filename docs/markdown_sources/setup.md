# How to setup {#setup}
[TOC]

## Installation
1. Follow the installation for the Arduino IDE + Dynamixel2Arduino library + CM9.04 board
2. Specific Arduino file organization

## Include in a project

### Specific file organization
Due to the file architecture enforced by Arduino, the file organization of your code **must** look like this:
![File tree](../img/folder_tree.png)

It is very straightforward to include this library in a project. \n
All the headers are located in "KMR_dxl/include", and, as already mentioned, the static libraries in "KMR_dxl/build".



In the source code, only one header needs to be included:
```cpp
#include "KMR_dxl_robot.hpp"
```

Next: how to [use](#how-to-use)