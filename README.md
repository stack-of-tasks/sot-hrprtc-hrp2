RTC component to encapsulte the SoT 
===================================

This RTC component will load a dynamic library containing
all the control algorithms to generate motion for a humanoid robot.

To install
===================================

mkdir _build-RELEASE

cd _build-RELEASE

cmake -DCMAKE_BUILD_TYPE=RELEASE ..

make

make install
