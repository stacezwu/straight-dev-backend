#!/bin/sh

  cat $1 \
| sed 's/\r//' \
| awk 'BEGIN{nops=0}{if($3=="NOP"){nops++}else if(nops==0){print}else{print " RPINC",nops-1;print;nops=0}}END{if(nops!=0){print " RPINC", nops-1}}' \
| awk '{ if( $0 !~ /^.\[.*(PHI|IMPLICIT)/ ) { print $0 } }' \
| awk 'BEGIN{print " [start ] JMP      Function_main"}{print}' \
| $(dirname $0)/linker \
| sed '/:$/d' \
| sed 's/^.\[......\]//g' \
| sed 's/#.*$//g' \
| sed 'y/<>,/   /' \
| sed 's/JMP/J  /' \
| sed 's/RMOV\(.*\)/ADDi.64 \1 0/'
