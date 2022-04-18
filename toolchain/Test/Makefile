SUBDIR := coremark dhrystone hello_musl
SUBDIR_all := $(addsuffix _all,$(SUBDIR))
SUBDIR_test := $(addsuffix _test,$(SUBDIR))
SUBDIR_rvtest := $(addsuffix _rvtest,$(SUBDIR))
SUBDIR_clean := $(addsuffix _clean,$(SUBDIR))

.PHONY : all $(SUBDIR_all) test $(SUBDIR_test) rvtest $(SUBDIR_rvtest) clean $(SUBDIR_clean)

all : $(SUBDIR_all)

$(SUBDIR_all) : %_all :
	cd $*; make all

test : $(SUBDIR_test)

$(SUBDIR_test) : %_test :
	cd $*; make test

rvtest : $(SUBDIR_rvtest)

$(SUBDIR_rvtest) : %_rvtest :
	cd $*; make rvtest


clean : $(SUBDIR_clean)

$(SUBDIR_clean) : %_clean :
	cd $*; make clean
