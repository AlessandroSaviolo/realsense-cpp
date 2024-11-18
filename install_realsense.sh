git clone https://github.com/IntelRealSense/librealsense.git --branch v2.55.1
cd librealsense
mkdir build
cd build
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install git wget cmake build-essential
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
cmake ../ -DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA:bool=true -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_OPENMP:bool=true
sudo make uninstall && make clean && make -j 8 && sudo make install
