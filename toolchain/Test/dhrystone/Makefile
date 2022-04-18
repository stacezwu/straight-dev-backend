include ../stuff/make.inc

LIB  :=lib/lputchar.s

.PHONY : all clean test check
.DEFAULT_GOAL=all

intermediate/dhry_1.ll intermediate/dhry_2.ll : intermediate/%.ll : src/%.c
	mkdir -p intermediate/
	$(clang12) --target=riscv64-pc_linux-gnu -DTIME -O2 -S -emit-llvm $< -o $@

intermediate/llibc.ll : intermediate/%.ll : lib/%.c
	mkdir -p intermediate/
	$(clang12) --target=riscv64-pc_linux-gnu -DTIME -O2 -S -emit-llvm $< -o $@


intermediate/dhrystone_linked.ll : intermediate/dhry_1.ll intermediate/dhry_2.ll intermediate/llibc.ll
	$(llvm-link12) $^ -S -o $@

intermediate/dhrystone.sz : intermediate/dhrystone_linked.ll
	$(llc) -march=straight -O2 $< -o $@ > $@.log 2>&1

# .szファイルは末尾に置く必要がある
intermediate/dhrystone.sl : $(LIB) intermediate/dhrystone.sz
	cat $^ > $@

intermediate/dhrystone.s : intermediate/dhrystone.sl ../../linker/linker
	../../linker/assemble_barebone.sh < $< > $@


stbin/dhrystone.stbin : intermediate/dhrystone.s ../../../sasm2/sasm2
	mkdir -p stbin/
	../../../sasm2/sasm2 --file $< --output $@

# 出力はどうせ環境依存なので、鬼斬の出力を丸ごと比較
dhrystone.stbin.result : stbin/dhrystone.stbin
	$(onikiri) stuff/param.xml > $@

all : dhrystone.stbin.result

clean :
	rm -rf intermediate/ stbin/ dhrystone.stbin.result dhrystone.stbin.xml

check : dhrystone.stbin.result
	diff -w -q stuff/ref.stbin.result $<

test : check
	@echo '  ___   _   _    ___    ___    ___   ___   ___ '
	@echo ' / __| | | | |  / __|  / __|  / _ \ / __| / __|'
	@echo ' \__ \ | |_| | | (__  | (__  |  __/ \__ \ \__ \'
	@echo ' |___/  \__,_|  \___|  \___|  \___| |___/ |___/'
