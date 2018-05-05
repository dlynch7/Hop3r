# ATLAS on the Raspberry Pi 3

Documenting my attempt to set up ATLAS on a Raspberry Pi 3.

## Get started

Source directory: `~/numerics/ATLAS3.10.3/`
Build directory: `~/numerics/ATLAS3.10.3/Build4Pi`

## The ATLAS configure step

### Configure flags
The Pi 3's CPU consists of 4 ARM Cortex-A53 cores, which are 64-bit; however, Raspbian (the standard Pi OS) is 32-bit. Therefore, ensure that you build with 32-bit libraries by configuring with
```
-b 32
```

The next one is weird. The Pi 3's CPU architecture is 64-bit, but the OS is 32-bit, so I guess we're stuck with 32-bit.

Don't do the following!!!
Tell `configure` to support ARMv7l architecture by adding this flag:
```
-A ARMv7l
```

Force ATLAS to use NEON (ARM32's SIMD vectorization):
```
-Fa al -mfpu=neon -Si ieee 0
```

Back to normal instructions now:

Building a full LAPACK library using ATLAS and netlib's LAPACK: if you want your final libraries to have all the LAPACK routines, then you just need to pass the `--with-netlib-lapack-tarfile` flag to configure, along with the netlib tarfile that you have previously downloaded.

`--with-netlib-lapack-tarfile=/home/pi/Downloads/lapack-3.8.0.tar.gz`

Possible concern: ATLAS 3.10.0 was tested to work with LAPACK v3.4.1 and 3.3.1. Will ATLAS work with LAPACK v3.8.0?

### Changing the compilers and flags that ATLAS uses for the build
Probably unnecessary

### FORTRAN on the Pi
ATLAS expects to find a FORTRAN compiler on your system.
```
sudo apt-get install gfortran
```

### Timer selection
The normal wall timer is still much more accurate than the CPU timer. Use it by passing the following flags to configure:
```
-D c -DWALL
```

Note: this flag somehow caused `make build` to fail, so I subsequently deleted it and reconfigured.

This flag works for non-x86 machines such as the Pi's ARMv7l.

### Building dynamic/shared libraries
ATLAS optionally builds a dynamic/shared library (ending in `.so`) if you pass the right compile flags. If you're using gnu compilers, pass `--shared`

### Putting all the flags together:
Run something like

```
../configure -b 32  --shared --with-netlib-lapack-tarfile=/home/pi/Downloads/lapack-3.8.0.tar.gz
```
