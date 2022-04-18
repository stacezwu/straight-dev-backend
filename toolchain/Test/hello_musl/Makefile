include ../stuff/make.inc

.PHONY : all clean test check
.DEFAULT_GOAL=all

intermediate/main.ll : src/main.c
	mkdir -p intermediate/
	$(clang12) --target=riscv64-pc_linux-gnu -O2 -S -emit-llvm $< -o $@

intermediate/hello_musl.ll : intermediate/main.ll $(linked_musl.ll)
	$(llvm-link12) $^ | $(opt12) -internalize -internalize-public-api-list=__libc_start_main,main,__cancel -globaldce -S -o $@

intermediate/hello_musl.sz : intermediate/hello_musl.ll
	$(llc) -march=straight -O2 $< -o $@ > $@.log 2>&1

intermediate/hello_musl.sl : $(asm_dir)/*.s intermediate/hello_musl.sz
	cat $^ > $@

intermediate/hello_musl.s : intermediate/hello_musl.sl
	../../linker/assemble_musl.sh < $< > $@

stbin/hello_musl.stbin : intermediate/hello_musl.s ../../../sasm2/sasm2
	mkdir -p stbin/
	../../../sasm2/sasm2 --file $< --output $@

hello_musl.stbin.result : stbin/hello_musl.stbin
	$(onikiri) stuff/param.xml > $@ 2> /dev/null

all : stbin/hello_musl.stbin

clean :
	rm -rf intermediate/ stbin/ hello_musl.stbin.result hello_musl.stbin.xml stuff/hello_musl.stbin.stdout

check : hello_musl.stbin.result
	diff -w -q stuff/hello_musl.stbin.stdout stuff/ref.stdout

test : check
	@echo '  ___   _   _    ___    ___    ___   ___   ___ '
	@echo ' / __| | | | |  / __|  / __|  / _ \ / __| / __|'
	@echo ' \__ \ | |_| | | (__  | (__  |  __/ \__ \ \__ \'
	@echo ' |___/  \__,_|  \___|  \___|  \___| |___/ |___/'
