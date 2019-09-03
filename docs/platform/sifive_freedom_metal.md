SiFive Freedom Metal Library Platform
=====================================
This platform uses freedom-metal library to abstract RISC-V platfroms.
Ideally, we don't consider what platform is going to run on it. It's
transparent by freedom-metal libary. Just make sure you go the right
device tree.

What is Freedom Metal
---------------------

Freedom Metal is a library which enables portable, bare-metal application
development for all of SiFive’s RISC-V IP, FPGA evaluation targets, and
development boards.

We get the benefits from Freedom Metal as following:
 - The ability to retarget to any SiFive RISC-V product
 - A RISC-V hardware abstraction layer
 - An API for controlling CPU features and peripherals

Freedom Metal is generally available from the Freedom Metal GitHub Repository
(https://github.com/sifive/freedom-metal)

How to Build Platform
---------------------

To build platform specific library and firmwares, provide the
*PLATFORM=sifive/freedom_metal* parameter to the top level `make` command.

Platform Options
----------------

Give the Freedom Metal library path. *FREEDOM_METAL_PATH* compile time option
is provided to specify where is your freedom metal library.

```
make PLATFORM=sifive/freedom_metal FW_PAYLOAD_PATH=Image FREEDOM_METAL_PATH=path
```

Building SiFive Freedom Metal Platform for FU540
------------------------------------------------

It makes no difference from building FU540 platform.
Please refer to FU540 documentation. (see: docs/platform/sifive_fu540.md)
