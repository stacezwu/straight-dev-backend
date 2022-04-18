include ../stuff/make.inc

LIB  :=lib/lputchar.s

.PHONY : all clean test check
.DEFAULT_GOAL=all

XCFLAGS := \
    -DCLOCKS_PER_SEC=1 \
    -DFLAGS_STR=\"\" \
    -DRUN_ON_64BIT

LLS := $(addprefix intermediate/,core_list_join.ll core_main.ll core_matrix.ll core_portme.ll core_state.ll core_util.ll)
LIBLL := $(addprefix intermediate/,memset.ll memcpy.ll puts.ll ee_printf.ll)

$(LIBLL) : intermediate/%.ll : src/%.c
	mkdir -p intermediate/
	$(clang12) $(XCFLAGS) --target=riscv64-pc_linux-gnu -O2 -S -emit-llvm $< -o $@

$(LLS) : intermediate/%.ll : src/%.c
	mkdir -p intermediate/
	$(clang12) -DITERATIONS=10 $(XCFLAGS) -DTIME --target=riscv64-pc_linux-gnu -O2 -S -emit-llvm $< -o $@


intermediate/coremark_linked.ll : $(LLS) $(LIBLL)
	$(llvm-link12) $^ -S -o $@

intermediate/coremark.sz : intermediate/coremark_linked.ll
	$(llc) -march=straight -O2 $< -o $@ > $@.log 2>&1

# .szを最後に持ってくる必要がある
intermediate/coremark.sl : $(LIB) intermediate/coremark.sz
	cat $^ > $@

intermediate/coremark.s : intermediate/coremark.sl ../../linker/linker
	../../linker/assemble_barebone.sh < $< > $@

stbin/coremark.stbin : intermediate/coremark.s ../../../sasm2/sasm2
	mkdir -p stbin/
	../../../sasm2/sasm2 --file $< --output $@

coremark.stbin.result : stbin/coremark.stbin
	$(onikiri) stuff/param.xml > $@ 2> /dev/null

all : coremark.stbin.result

clean :
	rm -rf intermediate/ stbin/ coremark.stbin.result coremark.stbin.xml

check : coremark.stbin.result
	diff -w -q stuff/ref.stbin.result $<

test : check
	@echo '  ___   _   _    ___    ___    ___   ___   ___ '
	@echo ' / __| | | | |  / __|  / __|  / _ \ / __| / __|'
	@echo ' \__ \ | |_| | | (__  | (__  |  __/ \__ \ \__ \'
	@echo ' |___/  \__,_|  \___|  \___|  \___| |___/ |___/'
