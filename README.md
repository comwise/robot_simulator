# robot_simulator
robot simulator, can simulate chassis and sensor, you can control chassis motion, safety avoid obstacle, navigation, multiple robot  path plan

# Depend
ubuntu 16.04
ros-kinetic ros1

# How to build
./script/install_env.sh
./script/build.sh
or
you can do the following:
make -p build && cd build
cmake ..
make

# Step
1. motion
2. map
3. nav(path plan)
4. nav(path trajectory)
5. test
6. gui

# R&A
todo