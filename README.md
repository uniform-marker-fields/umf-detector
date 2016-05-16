UMF detector library
=======================

In case you end up using our source code in research, please add a reference to one of our relevant publications:

Binary UMF detection:

I. Szentandrasi, M. Zacharias, J. Havel, A. Herout, M. Dubska, and R. Kajan. Uniform Marker
Fields: Camera localization by orientable De Bruijn tori. In ISMAR 2012, 2012

Grayscale UMF detection:

Adam Herout, Istvan Szentandrasi, Michal Zacharias, Marketa Dubska, and Rudolf Kajan.
Five shades of grey for fast and reliable camera pose estimation. In Proceedings of CVPR,
pages 1384–1390. IEEE Computer Society, 2013

Unity plugin or Chromakeying UMF:

I. Szentandrasi, M. Dubska, M. Zacharias, and A. Herout. Poor man’s virtual camera:
Real-time simultaneous matting and camera pose estimation. IEEE Computer Graphics and
Applications, 2016

Compilation - linux
---------------------------------

Dependencies
- argtable2
- opencv [optional | recommended]
- libdc1394-2 [optional] - firewire support
- Halide [optional] - optimized code

Included
- Eigen3
- glm

```bash
$ touch .depend
$ make depend
$ make all
```

To generate halide 
```bash
$ cd ext/halide
$ export HALIDE_PATH=/your/path/to/halide
$ make all
```

This generates assembly code for used for linux (you should add the USE_NATIVE in defines.h, and add the ext/lib/native_x.a to Makefile) and assembly files for android build.

Related Projects
----------------------------------------

* [UMF generator server](https://github.com/szist/umf-generator-server)
* [UMF generator client](https://github.com/szist/umf-generator-client)
* [UMF unity plugin] (https://github.com/szist/umf-unity)
