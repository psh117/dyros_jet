# DYROS JET Repository #

* This is a DYROS JET Humanoid Repository

## How do I get set up? ##

```sh
$ sudo apt-get install ros-kinetic-qt-build
```

### RBDL Setup ###
```sh
$ wget https://bitbucket.org/rbdl/rbdl/get/default.zip
$ unzip default.zip
$ cd rbdl-rbdl-849d2aee8f4c
$ mkdir build
$ cd build
$ cmake -DRBDL_BUILD_ADDON_URDFREADER=ON ..
$ make all
$ sudo make install
```
