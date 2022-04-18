Function___clone :
 [ ctid ] IMPLICIT_ARG_VALUE
 [ tls  ] IMPLICIT_ARG_VALUE
 [ ptid ] IMPLICIT_ARG_VALUE
 [ arg  ] IMPLICIT_ARG_VALUE
 [ flags] IMPLICIT_ARG_VALUE
 [ stack] IMPLICIT_ARG_VALUE
 [ func ] IMPLICIT_ARG_VALUE
 [RETADR] IMPLICIT_RET_ADDR
 SPADDi -16
 SPST.64 2 8 # save RETADDR
 ST.64 4 5 -16 # save func to stack
 ST.64 8 6 -8 # save arg to stack
 RMOV 12 # ctid
 RMOV 12 # tls
 RMOV 12 # ptid
 ADDi.64 10 16 # stack
 RMOV 12 # flags
 ADDi.64 0 220 # SYS_clone
 ECALL
 BEQ 0 1 8
 SPLD.64 8
 RMOV 3
 JR 2 0
 SPLD.64 0
 SPLD.64 8
 JALR 2 0
 ADDi.64 0 93 # SYS_exit
 ECALL
