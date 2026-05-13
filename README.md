# Robotic arm simulation webots

## Setting up enviornment

### Putting files in the right place

For the simulation to work, the folder "robotic_arm_model" must be copied and pasted into the libraries folder in the webots home folder.

The MyProto.proto file also has to be copy pasted into the protos folder for webots to recognize it.

## Building/running control

First you need to set up a cmake build folder:

```
mkdir ./build
cmake -S . -B ./build
```

and can be built by using
```
cmake --build ./build
```

For webots to recognize the controller, there also needs to be a simlink to the target made in the build folder:
```
ln -s ./build/my_arm_controller ./robotic-arm-webots-simulation
```
