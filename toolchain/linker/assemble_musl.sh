#!/bin/sh

  cat $1 \
| sed 's/\r//' \
| awk 'BEGIN{nops=0}{if($3=="NOP"){nops++}else if(nops==0){print}else{print " RPINC",nops-1;print;nops=0}}END{if(nops!=0){print " RPINC", nops-1}}' \
| awk '{ if( $0 !~ /^.\[.*(PHI|IMPLICIT)/ ) { print $0 } }' \
| awk 'BEGIN{print " [start ] Global   main\n [start ] Global   __lib_csu_init\n [start ] Global   __lib_csu_fini\n [ arg6 ] AUiSP 0\n [ arg5 ] ADDi.64 0 0\n [ arg4 ] RMOV 3\n [ arg3 ] RMOV 6\n [ arg2 ] ADDi.64 4 8\n [ arg1 ] SPLD.64 0\n [ arg0 ] RMOV 11\n [start ] JAL      Function___libc_start_main"}{print}' \
| $(dirname $0)/linker \
| sed '/:$/d' \
| sed 's/^.\[......\]//g' \
| sed 's/#.*$//g' \
| sed 'y/<>,/   /' \
| sed 's/JMP/J  /' \
| sed 's/RMOV\(.*\)/ADDi.64 \1 0/'
