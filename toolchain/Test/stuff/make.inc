# Set your environment
llc:= ../../../ninja/bin/llc
onikiri:= ../../../onikiri2/project/gcc/onikiri2/a.out

# LLVM12 prebuild binaries
clang12 := ../../../clang+llvm-12.0.1-x86_64-linux-gnu-ubuntu-16.04/bin/clang
llvm-link12 := ../../../clang+llvm-12.0.1-x86_64-linux-gnu-ubuntu-16.04/bin/llvm-link
opt12 := ../../../clang+llvm-12.0.1-x86_64-linux-gnu-ubuntu-16.04/bin/opt
llc12 := ../../../clang+llvm-12.0.1-x86_64-linux-gnu-ubuntu-16.04/bin/llc

linked_musl.ll := ../../build-musl/linked_musl.ll
asm_dir := ../../asm

ifeq ($(shell ls $(onikiri)),)
$(error cannot find onikiri; check the configuration in toolchain/Test/stuff/make.inc)
endif
ifeq ($(shell ls $(llc)),)
$(error cannot find llc; check the configuration in toolchain/Test/stuff/make.inc)
endif
ifeq ($(shell ls $(clang12)),)
$(error cannot find clang12; check the configuration in toolchain/Test/stuff/make.inc)
endif
ifeq ($(shell ls $(llvm-link12)),)
$(error cannot find llvm-link12; check the configuration in toolchain/Test/stuff/make.inc)
endif
ifeq ($(shell ls $(opt12)),)
$(error cannot find opt12; check the configuration in toolchain/Test/stuff/make.inc)
endif
ifeq ($(shell ls $(llc12)),)
$(error cannot find llc12; check the configuration in toolchain/Test/stuff/make.inc)
endif

../../../sasm2/sasm2 : ../../../sasm2/*.*
	cd ../../../sasm2; go build

../../linker/linker : ../../linker/*.*
	cd ../../linker; make

../../build-musl/linked_musl.ll :
	cd ../../build-musl; make
