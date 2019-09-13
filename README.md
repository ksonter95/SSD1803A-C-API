# SSD1803A-C-API
C library for the SSD1803A LCB Display MPU

## Installation ##
To compile the source code into the static (libssd1803a.a) and shared (libssd1803a.so) libraries, simply run the command:
```
make
```
To install the libraries, simply run the command shown below, which will install the libraries into '/usr/local/lib' and the header file into '/usr/local/include':
```
make install
```
If at any point the libraries are to be uninstalled, simply run the command shown below, which will uninstall the libraries from '/usr/local/lib' and the header files from '/usr/local/include':
```
make uninstall
```

## Usage ##
To use the library in other programmes, follow these three steps:
1. In the programme source files that use the library, include the line:
```
#include <ssd1803a.h>
```
2. When compiling the source code into the object files, link in the library using the following command, where foo.c is to be replaced with the name of the source file than uses the library:
```
gcc foo.c -lssd1803a
```
3. This step may be unnecessary, but it is required in the event that a 'error while loading shared libraries: libssd1803a.so: cannot open shared object file: No such file or directory' error is thrown when the executable is attempted to be run.  This creates the necessary links and cache to the most recent shared libraries found in the directories specified on the command line, in the file /etc/ld.so.conf, and in the trusted directories, /lib, and /usr/lib.
```
ldconfig
```