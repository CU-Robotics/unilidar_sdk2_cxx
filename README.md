# unilidar_sdk2_cxx

Rust bindings for the unilidar_sdk2 C++ library for the Unitree L2 LIDAR.

`/unitree_lidar_sdk` is the original C++ library, copied from the `/unitree_lidar_sdk` folder of https://github.com/unitreerobotics/unilidar_sdk2

The vendored SDK matches Unitree's `v2.0.10` tag (`0e3c51f`). Note that the precompiled `libunilidar_sdk2.a` in that tag still reports the internal SDK version string `2.0.9`.

The C++ library is shipped as precompiled aarch64 and x86_64 binaries. A small set of header files is given with class/struct definitions.

I tried my best to infer what each definition means, but the documentation is lackluster, so I had to make a number of assumptions. I've done my best to document every assumption made. Further testing needs to be done to figure out the order of structs, as well as what packets can be returned by the lidar.

sudo ip addr add 192.168.1.2/24 dev enp195s0f3u1u3
sudo ip link set enp195s0f3u1u3 up
ip route get 192.168.1.62 # should now show "dev enp195s0f3u1u3", not via 10.233.x.x
ping -c 2 192.168.1.62
