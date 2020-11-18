# spot_micro_kinematics_cpp - ESP_DSP
A c++ library for kinematic operations on a spot micro quadruped

Ported for ESP platform, and implemented using ESP_DSP
Use the library as a ESP-IDF component.

# Requirement
ESP-DSP component: https://github.com/espressif/esp-dsp.git


# Component
Create component with <b>CMakeLists.txt</b> as below
```cmake
idf_component_register(SRCS spot_micro_kinematics_cpp/src/spot_micro_kinematics.cpp spot_micro_kinematics_cpp/src/spot_micro_leg.cpp spot_micro_kinematics_cpp/src/utils.cpp
                  INCLUDE_DIRS spot_micro_kinematics_cpp/include
                  PRIV_REQUIRES esp-dsp)
```
checkout the reposity under the ESP-IDF component folder (eg: <project>/components/kinematics/spot_micro_kinematics_cpp)
