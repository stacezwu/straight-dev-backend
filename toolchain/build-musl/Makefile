musl:=../../musl
clang12:=$(shell pwd)/../../clang+llvm-12.0.1-x86_64-linux-gnu-ubuntu-16.04/bin

# long double 用の関数はコンパイルできないので除去
# 基本的には***l.oを除去すればいいが、ceil.oだけは例外


.PHONY: all clean

all : linked_musl.ll

clean :
	rm $(musl)/config.mak linked_musl.ll
	cd $(musl); make clean

linked_musl.ll : $(musl)/config.mak
	cd $(musl); make -k || true # llvmIRを出力しているので当然リンクできずエラーになるが問題ない
	cd $(musl); make obj/src/thread/__unmapself.o obj/src/thread/__set_thread_area.o # RISC-Vだと代替アセンブリがあるためコンパイルされないが、強制的にmakeすればコンパイルできる
	$(clang12)/llvm-link $$(ls $(musl)/obj/src/*/*.o $(musl)/obj/src/*/riscv64/*.o | grep -v 'math/.*l\.o' | grep -v 'complex/.*\.o' | grep -v 'nexttoward.*\.o' | grep -v 'siglongjmp\.o' | grep -v 'time/timer_create\.o') $(musl)/obj/src/math/ceil.o -S > $@

$(musl)/config.mak :
	cd $(musl); CC=$(clang12)/clang CROSS_COMPILE=foo CFLAGS='--target=riscv64-pc_linux-gnu -S -emit-llvm' ./configure --prefix=../p --target=riscv64

