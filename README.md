# Setup Enviroment Section

In order to use this repository, the user have to install the following
resurces and programs.

## Install Toolchain Prerequisities

Some of the requirments for iInstalating and using this software are :
- Update the system
- Instalation GitHub
- Instalationl Autoreconf & LibUSB 
- Instalation Cmake
- Instalation gtk


## Install Toolchain

### Installation Compiler and Debbuger

#### ARM Developer

The GNU Embedded Toolchain for Arm is a ready-to-use, open source suite of
tools for C, C++ and Assembly programming targeting Arm Cortex-M and Cortex-R
family of processors. 

The PPA we want in this case is from the GCC ARM Embedded Maintainer’s
team.

```
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install gcc-arm-embedded
```
#### Debugger

Bare metal GNU debugger for embedded ARM chips using Cortex-M0/M0+/M3/M4,
Cortex-R4/R5/R7 and Cortex-A* processors. GDB is a source-level debugger,
capable of breaking programs at any specific line, displaying variable values,
and determining where errors occurred.

```
sudo apt-get install gdb-arm-none-eabi
```

If it installs correctly, doing autocomplete in the terminal like so:

```
arm-none (press tab twice)
```

You should look something similar like:

```
arm-none-eabi-addr2line   arm-none-eabi-gcc-ar      arm-none-eabi-nm
arm-none-eabi-ar          arm-none-eabi-gcc-nm      arm-none-eabi-objcopy
arm-none-eabi-as          arm-none-eabi-gcc-ranlib  arm-none-eabi-objdump
arm-none-eabi-c++         arm-none-eabi-gcov        arm-none-eabi-ranlib
arm-none-eabi-c++filt     arm-none-eabi-gcov-dump   arm-none-eabi-readelf
arm-none-eabi-cpp         arm-none-eabi-gcov-tool   arm-none-eabi-size
arm-none-eabi-elfedit     arm-none-eabi-gdb         arm-none-eabi-strings
arm-none-eabi-g++         arm-none-eabi-gprof       arm-none-eabi-strip
arm-none-eabi-gcc         arm-none-eabi-ld          
arm-none-eabi-gcc-7.3.1   arm-none-eabi-ld.bfd 
```

###  OpenOCD

OpenOCD is a free on chip debug solution for targets based on ARM. It is a server
which opens a GDB remote target port and a Telnet port.  
  
In order to install OpenOCD, the code necesary for instalation could be found
in [OpenOCD](https://sourceforge.net/projects/openocd/files/openocd/0.10.0/).

The following comands generate and install the program. 

```
 ./configure [options]
 make
 sudo make install
```

### Installation St-link

TLink package is the software for the STLink programmer that works with many
boards ST boards. On the discovery boards, the programmer is embedded at the
top. YOu just need to make sure you plug in your mini-USB cable into the
center-most port that is labeled ST-Link.

First, clone the repository st-link.

```
git clone https://github.com/texane/stlink.git
```

Every thing can built from the top of directory.

```
cd stlink
make
cd build/Release && make install DESTDIR=_install
```
