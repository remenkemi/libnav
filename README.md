# libnav

This library provides parsers for x-plane's CIFP, earth_hold.dat, earth_awy.dat, earth_fix.dat, earth_nav.dat and apt.dat files. The library also provides various utility functions for working with lat,lon points(great circle distance, bearing, pbd points and more) and strings(decimal lat/lon to dms, etc). Each data base is loaded in its own thread. This means your program can also perform other tasks while the data bases are loading. Each data base interface is implemented in a thread-safe manner.

## Getting started

The documentation for each symbol is located in its respective .hpp file. Most of the symbols have been documented by now. Documentation using doxygen is planned. There is also a demo project available [here](https://github.com/BRUHegg/libnav-demo). The compilation process for the demo is analogous to how you'd compile this library.

## Compiling

This is a static library. It's compiled as part of a demo app that allows you to interface x-plane's data bases. For more info look at the main.cpp file located in src/test
### Compiling on linux/mac
To compile you will need to create a directory called build(inside the repository's directory) and in that call cmake .. and make. Like in the example below.
```text
cd {your path to libnav repository}
mkdir build
cd build
cmake ..
make
```
### Compiling on windows using mingw:
The process of compiling this with mingw is similar to how you'd compile it on linux. See the sequence of commands below:
```text
cd {your path to libnav repository}
mkdir build
cd build
cmake .. -G "MSYS Makefiles"
cmake --build .
```
After compilation the library itself should be in your libnav/build/src/libnav directory. The file should be called something like liblibnav.a

## Using pre-compiled version
If you don't want to compile the library yourself, you can download the pre-compiled version [here](https://github.com/BRUHegg/libnav-redist/).
