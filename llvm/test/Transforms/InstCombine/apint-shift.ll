; NOTE: Assertions have been autogenerated by utils/update_test_checks.py
; RUN: opt < %s -instcombine -S | FileCheck %s

define i55 @test6(i55 %A) {
; CHECK-LABEL: @test6(
; CHECK-NEXT:    [[C:%.*]] = mul i55 [[A:%.*]], 6
; CHECK-NEXT:    ret i55 [[C]]
;
  %B = shl i55 %A, 1
  %C = mul i55 %B, 3
  ret i55 %C
}

; (X * C2) << C1 --> X * (C2 << C1)

define i55 @test6a(i55 %A) {
; CHECK-LABEL: @test6a(
; CHECK-NEXT:    [[C:%.*]] = mul i55 [[A:%.*]], 6
; CHECK-NEXT:    ret i55 [[C]]
;
  %B = mul i55 %A, 3
  %C = shl i55 %B, 1
  ret i55 %C
}

; (X * C2) << C1 --> X * (C2 << C1)

define <2 x i55> @test6a_vec(<2 x i55> %A) {
; CHECK-LABEL: @test6a_vec(
; CHECK-NEXT:    [[C:%.*]] = mul <2 x i55> [[A:%.*]], <i55 6, i55 48>
; CHECK-NEXT:    ret <2 x i55> [[C]]
;
  %B = mul <2 x i55> %A, <i55 3, i55 12>
  %C = shl <2 x i55> %B, <i55 1, i55 2>
  ret <2 x i55> %C
}

define i29 @test7(i8 %X) {
; CHECK-LABEL: @test7(
; CHECK-NEXT:    ret i29 -1
;
  %A = zext i8 %X to i29
  %B = ashr i29 -1, %A
  ret i29 %B
}

define i7 @test8(i7 %A) {
; CHECK-LABEL: @test8(
; CHECK-NEXT:    ret i7 0
;
  %B = shl i7 %A, 4
  %C = shl i7 %B, 3
  ret i7 %C
}

define i17 @test9(i17 %A) {
; CHECK-LABEL: @test9(
; CHECK-NEXT:    [[B:%.*]] = and i17 [[A:%.*]], 1
; CHECK-NEXT:    ret i17 [[B]]
;
  %B = shl i17 %A, 16
  %C = lshr i17 %B, 16
  ret i17 %C
}

; shl (lshr X, C), C --> and X, C'

define i19 @test10(i19 %X) {
; CHECK-LABEL: @test10(
; CHECK-NEXT:    [[SH1:%.*]] = and i19 [[X:%.*]], -262144
; CHECK-NEXT:    ret i19 [[SH1]]
;
  %sh1 = lshr i19 %X, 18
  %sh2 = shl i19 %sh1, 18
  ret i19 %sh2
}

; Two right shifts in the same direction:
; lshr (lshr X, C1), C2 --> lshr X, C1 + C2

define <2 x i19> @lshr_lshr_splat_vec(<2 x i19> %X) {
; CHECK-LABEL: @lshr_lshr_splat_vec(
; CHECK-NEXT:    [[SH1:%.*]] = lshr <2 x i19> [[X:%.*]], <i19 5, i19 5>
; CHECK-NEXT:    ret <2 x i19> [[SH1]]
;
  %sh1 = lshr <2 x i19> %X, <i19 3, i19 3>
  %sh2 = lshr <2 x i19> %sh1, <i19 2, i19 2>
  ret <2 x i19> %sh2
}

define i9 @multiuse_lshr_lshr(i9 %x) {
; CHECK-LABEL: @multiuse_lshr_lshr(
; CHECK-NEXT:    [[SH1:%.*]] = lshr i9 [[X:%.*]], 2
; CHECK-NEXT:    [[SH2:%.*]] = lshr i9 [[X]], 5
; CHECK-NEXT:    [[MUL:%.*]] = mul i9 [[SH1]], [[SH2]]
; CHECK-NEXT:    ret i9 [[MUL]]
;
  %sh1 = lshr i9 %x, 2
  %sh2 = lshr i9 %sh1, 3
  %mul = mul i9 %sh1, %sh2
  ret i9 %mul
}

define <2 x i9> @multiuse_lshr_lshr_splat(<2 x i9> %x) {
; CHECK-LABEL: @multiuse_lshr_lshr_splat(
; CHECK-NEXT:    [[SH1:%.*]] = lshr <2 x i9> [[X:%.*]], <i9 2, i9 2>
; CHECK-NEXT:    [[SH2:%.*]] = lshr <2 x i9> [[X]], <i9 5, i9 5>
; CHECK-NEXT:    [[MUL:%.*]] = mul <2 x i9> [[SH1]], [[SH2]]
; CHECK-NEXT:    ret <2 x i9> [[MUL]]
;
  %sh1 = lshr <2 x i9> %x, <i9 2, i9 2>
  %sh2 = lshr <2 x i9> %sh1, <i9 3, i9 3>
  %mul = mul <2 x i9> %sh1, %sh2
  ret <2 x i9> %mul
}

; Two left shifts in the same direction:
; shl (shl X, C1), C2 -->  shl X, C1 + C2

define <2 x i19> @shl_shl_splat_vec(<2 x i19> %X) {
; CHECK-LABEL: @shl_shl_splat_vec(
; CHECK-NEXT:    [[SH1:%.*]] = shl <2 x i19> [[X:%.*]], <i19 5, i19 5>
; CHECK-NEXT:    ret <2 x i19> [[SH1]]
;
  %sh1 = shl <2 x i19> %X, <i19 3, i19 3>
  %sh2 = shl <2 x i19> %sh1, <i19 2, i19 2>
  ret <2 x i19> %sh2
}

define i42 @multiuse_shl_shl(i42 %x) {
; CHECK-LABEL: @multiuse_shl_shl(
; CHECK-NEXT:    [[SH1:%.*]] = shl i42 [[X:%.*]], 8
; CHECK-NEXT:    [[SH2:%.*]] = shl i42 [[X]], 17
; CHECK-NEXT:    [[MUL:%.*]] = mul i42 [[SH1]], [[SH2]]
; CHECK-NEXT:    ret i42 [[MUL]]
;
  %sh1 = shl i42 %x, 8
  %sh2 = shl i42 %sh1, 9
  %mul = mul i42 %sh1, %sh2
  ret i42 %mul
}

define <2 x i42> @multiuse_shl_shl_splat(<2 x i42> %x) {
; CHECK-LABEL: @multiuse_shl_shl_splat(
; CHECK-NEXT:    [[SH1:%.*]] = shl <2 x i42> [[X:%.*]], <i42 8, i42 8>
; CHECK-NEXT:    [[SH2:%.*]] = shl <2 x i42> [[X]], <i42 17, i42 17>
; CHECK-NEXT:    [[MUL:%.*]] = mul <2 x i42> [[SH1]], [[SH2]]
; CHECK-NEXT:    ret <2 x i42> [[MUL]]
;
  %sh1 = shl <2 x i42> %x, <i42 8, i42 8>
  %sh2 = shl <2 x i42> %sh1, <i42 9, i42 9>
  %mul = mul <2 x i42> %sh1, %sh2
  ret <2 x i42> %mul
}

; Equal shift amounts in opposite directions become bitwise 'and':
; lshr (shl X, C), C --> and X, C'

define <2 x i19> @eq_shl_lshr_splat_vec(<2 x i19> %X) {
; CHECK-LABEL: @eq_shl_lshr_splat_vec(
; CHECK-NEXT:    [[SH1:%.*]] = and <2 x i19> [[X:%.*]], <i19 65535, i19 65535>
; CHECK-NEXT:    ret <2 x i19> [[SH1]]
;
  %sh1 = shl <2 x i19> %X, <i19 3, i19 3>
  %sh2 = lshr <2 x i19> %sh1, <i19 3, i19 3>
  ret <2 x i19> %sh2
}

; Equal shift amounts in opposite directions become bitwise 'and':
; shl (lshr X, C), C --> and X, C'

define <2 x i19> @eq_lshr_shl_splat_vec(<2 x i19> %X) {
; CHECK-LABEL: @eq_lshr_shl_splat_vec(
; CHECK-NEXT:    [[SH1:%.*]] = and <2 x i19> [[X:%.*]], <i19 -8, i19 -8>
; CHECK-NEXT:    ret <2 x i19> [[SH1]]
;
  %sh1 = lshr <2 x i19> %X, <i19 3, i19 3>
  %sh2 = shl <2 x i19> %sh1, <i19 3, i19 3>
  ret <2 x i19> %sh2
}

; In general, we would need an 'and' for this transform, but the masked-off bits are known zero.
; shl (lshr X, C1), C2 --> lshr X, C1 - C2

define <2 x i7> @lshr_shl_splat_vec(<2 x i7> %X) {
; CHECK-LABEL: @lshr_shl_splat_vec(
; CHECK-NEXT:    [[MUL:%.*]] = mul <2 x i7> [[X:%.*]], <i7 -8, i7 -8>
; CHECK-NEXT:    [[SH1:%.*]] = lshr exact <2 x i7> [[MUL]], <i7 1, i7 1>
; CHECK-NEXT:    ret <2 x i7> [[SH1]]
;
  %mul = mul <2 x i7> %X, <i7 -8, i7 -8>
  %sh1 = lshr exact <2 x i7> %mul, <i7 3, i7 3>
  %sh2 = shl nuw nsw <2 x i7> %sh1, <i7 2, i7 2>
  ret <2 x i7> %sh2
}

; In general, we would need an 'and' for this transform, but the masked-off bits are known zero.
; lshr (shl X, C1), C2 -->  shl X, C1 - C2

define <2 x i7> @shl_lshr_splat_vec(<2 x i7> %X) {
; CHECK-LABEL: @shl_lshr_splat_vec(
; CHECK-NEXT:    [[DIV:%.*]] = udiv <2 x i7> [[X:%.*]], <i7 9, i7 9>
; CHECK-NEXT:    [[SH1:%.*]] = shl nuw nsw <2 x i7> [[DIV]], <i7 1, i7 1>
; CHECK-NEXT:    ret <2 x i7> [[SH1]]
;
  %div = udiv <2 x i7> %X, <i7 9, i7 9>
  %sh1 = shl nuw <2 x i7> %div, <i7 3, i7 3>
  %sh2 = lshr exact <2 x i7> %sh1, <i7 2, i7 2>
  ret <2 x i7> %sh2
}

define i23 @test11(i23 %x) {
; CHECK-LABEL: @test11(
; CHECK-NEXT:    [[TMP1:%.*]] = mul i23 [[X:%.*]], 6
; CHECK-NEXT:    [[C:%.*]] = and i23 [[TMP1]], -4096
; CHECK-NEXT:    ret i23 [[C]]
;
  %a = mul i23 %x, 3
  %b = lshr i23 %a, 11
  %c = shl i23 %b, 12
  ret i23 %c
}

; shl (ashr X, C), C --> and X, C'

define i47 @test12(i47 %X) {
; CHECK-LABEL: @test12(
; CHECK-NEXT:    [[TMP1:%.*]] = and i47 [[X:%.*]], -256
; CHECK-NEXT:    ret i47 [[TMP1]]
;
  %sh1 = ashr i47 %X, 8
  %sh2 = shl i47 %sh1, 8
  ret i47 %sh2
}

define <2 x i47> @test12_splat_vec(<2 x i47> %X) {
; CHECK-LABEL: @test12_splat_vec(
; CHECK-NEXT:    [[TMP1:%.*]] = and <2 x i47> [[X:%.*]], <i47 -256, i47 -256>
; CHECK-NEXT:    ret <2 x i47> [[TMP1]]
;
  %sh1 = ashr <2 x i47> %X, <i47 8, i47 8>
  %sh2 = shl <2 x i47> %sh1, <i47 8, i47 8>
  ret <2 x i47> %sh2
}

define i18 @test13(i18 %x) {
; CHECK-LABEL: @test13(
; CHECK-NEXT:    [[TMP1:%.*]] = mul i18 [[X:%.*]], 6
; CHECK-NEXT:    [[C:%.*]] = and i18 [[TMP1]], -512
; CHECK-NEXT:    ret i18 [[C]]
;
  %a = mul i18 %x, 3
  %b = ashr i18 %a, 8
  %c = shl i18 %b, 9
  ret i18 %c
}

define i35 @test14(i35 %A) {
; CHECK-LABEL: @test14(
; CHECK-NEXT:    [[B:%.*]] = and i35 [[A:%.*]], -19760
; CHECK-NEXT:    [[C:%.*]] = or i35 [[B]], 19744
; CHECK-NEXT:    ret i35 [[C]]
;
  %B = lshr i35 %A, 4
  %C = or i35 %B, 1234
  %D = shl i35 %C, 4
  ret i35 %D
}

define i79 @test14a(i79 %A) {
; CHECK-LABEL: @test14a(
; CHECK-NEXT:    [[C:%.*]] = and i79 [[A:%.*]], 77
; CHECK-NEXT:    ret i79 [[C]]
;
  %B = shl i79 %A, 4
  %C = and i79 %B, 1234
  %D = lshr i79 %C, 4
  ret i79 %D
}

define i45 @test15(i1 %C) {
; CHECK-LABEL: @test15(
; CHECK-NEXT:    [[A:%.*]] = select i1 [[C:%.*]], i45 12, i45 4
; CHECK-NEXT:    ret i45 [[A]]
;
  %A = select i1 %C, i45 3, i45 1
  %V = shl i45 %A, 2
  ret i45 %V
}

define i53 @test15a(i1 %X) {
; CHECK-LABEL: @test15a(
; CHECK-NEXT:    [[V:%.*]] = select i1 [[X:%.*]], i53 512, i53 128
; CHECK-NEXT:    ret i53 [[V]]
;
  %A = select i1 %X, i8 3, i8 1
  %B = zext i8 %A to i53
  %V = shl i53 64, %B
  ret i53 %V
}

define i1 @test16(i84 %X) {
; CHECK-LABEL: @test16(
; CHECK-NEXT:    [[TMP1:%.*]] = and i84 [[X:%.*]], 16
; CHECK-NEXT:    [[CMP:%.*]] = icmp ne i84 [[TMP1]], 0
; CHECK-NEXT:    ret i1 [[CMP]]
;
  %shr = ashr i84 %X, 4
  %and = and i84 %shr, 1
  %cmp = icmp ne i84 %and, 0
  ret i1 %cmp
}

define <2 x i1> @test16vec(<2 x i84> %X) {
; CHECK-LABEL: @test16vec(
; CHECK-NEXT:    [[TMP1:%.*]] = and <2 x i84> [[X:%.*]], <i84 16, i84 16>
; CHECK-NEXT:    [[CMP:%.*]] = icmp ne <2 x i84> [[TMP1]], zeroinitializer
; CHECK-NEXT:    ret <2 x i1> [[CMP]]
;
  %shr = ashr <2 x i84> %X, <i84 4, i84 4>
  %and = and <2 x i84> %shr, <i84 1, i84 1>
  %cmp = icmp ne <2 x i84> %and, zeroinitializer
  ret <2 x i1> %cmp
}

define <2 x i1> @test16vec_nonuniform(<2 x i84> %X) {
; CHECK-LABEL: @test16vec_nonuniform(
; CHECK-NEXT:    [[TMP1:%.*]] = and <2 x i84> [[X:%.*]], <i84 16, i84 4>
; CHECK-NEXT:    [[CMP:%.*]] = icmp ne <2 x i84> [[TMP1]], zeroinitializer
; CHECK-NEXT:    ret <2 x i1> [[CMP]]
;
  %shr = ashr <2 x i84> %X, <i84 4, i84 2>
  %and = and <2 x i84> %shr, <i84 1, i84 1>
  %cmp = icmp ne <2 x i84> %and, zeroinitializer
  ret <2 x i1> %cmp
}

define <2 x i1> @test16vec_undef(<2 x i84> %X) {
; CHECK-LABEL: @test16vec_undef(
; CHECK-NEXT:    [[TMP1:%.*]] = and <2 x i84> [[X:%.*]], <i84 16, i84 undef>
; CHECK-NEXT:    [[CMP:%.*]] = icmp ne <2 x i84> [[TMP1]], zeroinitializer
; CHECK-NEXT:    ret <2 x i1> [[CMP]]
;
  %shr = ashr <2 x i84> %X, <i84 4, i84 undef>
  %and = and <2 x i84> %shr, <i84 1, i84 1>
  %cmp = icmp ne <2 x i84> %and, zeroinitializer
  ret <2 x i1> %cmp
}

define i1 @test17(i106 %A) {
; CHECK-LABEL: @test17(
; CHECK-NEXT:    [[B_MASK:%.*]] = and i106 [[A:%.*]], -8
; CHECK-NEXT:    [[C:%.*]] = icmp eq i106 [[B_MASK]], 9872
; CHECK-NEXT:    ret i1 [[C]]
;
  %B = lshr i106 %A, 3
  %C = icmp eq i106 %B, 1234
  ret i1 %C
}

define <2 x i1> @test17vec(<2 x i106> %A) {
; CHECK-LABEL: @test17vec(
; CHECK-NEXT:    [[B_MASK:%.*]] = and <2 x i106> [[A:%.*]], <i106 -8, i106 -8>
; CHECK-NEXT:    [[C:%.*]] = icmp eq <2 x i106> [[B_MASK]], <i106 9872, i106 9872>
; CHECK-NEXT:    ret <2 x i1> [[C]]
;
  %B = lshr <2 x i106> %A, <i106 3, i106 3>
  %C = icmp eq <2 x i106> %B, <i106 1234, i106 1234>
  ret <2 x i1> %C
}

define i1 @test18(i11 %A) {
; CHECK-LABEL: @test18(
; CHECK-NEXT:    ret i1 false
;
  %B = lshr i11 %A, 10
  %C = icmp eq i11 %B, 123
  ret i1 %C
}

define i1 @test19(i37 %A) {
; CHECK-LABEL: @test19(
; CHECK-NEXT:    [[C:%.*]] = icmp ult i37 [[A:%.*]], 4
; CHECK-NEXT:    ret i1 [[C]]
;
  %B = ashr i37 %A, 2
  %C = icmp eq i37 %B, 0
  ret i1 %C
}

define <2 x i1> @test19vec(<2 x i37> %A) {
; CHECK-LABEL: @test19vec(
; CHECK-NEXT:    [[C:%.*]] = icmp ult <2 x i37> [[A:%.*]], <i37 4, i37 4>
; CHECK-NEXT:    ret <2 x i1> [[C]]
;
  %B = ashr <2 x i37> %A, <i37 2, i37 2>
  %C = icmp eq <2 x i37> %B, zeroinitializer
  ret <2 x i1> %C
}

define i1 @test19a(i39 %A) {
; CHECK-LABEL: @test19a(
; CHECK-NEXT:    [[C:%.*]] = icmp ugt i39 [[A:%.*]], -5
; CHECK-NEXT:    ret i1 [[C]]
;
  %B = ashr i39 %A, 2
  %C = icmp eq i39 %B, -1
  ret i1 %C
}

define <2 x i1> @test19a_vec(<2 x i39> %A) {
; CHECK-LABEL: @test19a_vec(
; CHECK-NEXT:    [[C:%.*]] = icmp ugt <2 x i39> [[A:%.*]], <i39 -5, i39 -5>
; CHECK-NEXT:    ret <2 x i1> [[C]]
;
  %B = ashr <2 x i39> %A, <i39 2, i39 2>
  %C = icmp eq <2 x i39> %B, <i39 -1, i39 -1>
  ret <2 x i1> %C
}

define i1 @test20(i13 %A) {
; CHECK-LABEL: @test20(
; CHECK-NEXT:    ret i1 false
;
  %B = ashr i13 %A, 12
  %C = icmp eq i13 %B, 123
  ret i1 %C
}

define i1 @test21(i12 %A) {
; CHECK-LABEL: @test21(
; CHECK-NEXT:    [[B_MASK:%.*]] = and i12 [[A:%.*]], 63
; CHECK-NEXT:    [[C:%.*]] = icmp eq i12 [[B_MASK]], 62
; CHECK-NEXT:    ret i1 [[C]]
;
  %B = shl i12 %A, 6
  %C = icmp eq i12 %B, -128
  ret i1 %C
}

define i1 @test22(i14 %A) {
; CHECK-LABEL: @test22(
; CHECK-NEXT:    [[B_MASK:%.*]] = and i14 [[A:%.*]], 127
; CHECK-NEXT:    [[C:%.*]] = icmp eq i14 [[B_MASK]], 0
; CHECK-NEXT:    ret i1 [[C]]
;
  %B = shl i14 %A, 7
  %C = icmp eq i14 %B, 0
  ret i1 %C
}

define i11 @test23(i44 %A) {
; CHECK-LABEL: @test23(
; CHECK-NEXT:    [[D:%.*]] = trunc i44 [[A:%.*]] to i11
; CHECK-NEXT:    ret i11 [[D]]
;
  %B = shl i44 %A, 33
  %C = ashr i44 %B, 33
  %D = trunc i44 %C to i11
  ret i11 %D
}

; Fold lshr (shl X, C), C -> and X, C' regardless of the number of uses of the shl.

define i44 @shl_lshr_eq_amt_multi_use(i44 %A) {
; CHECK-LABEL: @shl_lshr_eq_amt_multi_use(
; CHECK-NEXT:    [[B:%.*]] = shl i44 [[A:%.*]], 33
; CHECK-NEXT:    [[C:%.*]] = and i44 [[A]], 2047
; CHECK-NEXT:    [[D:%.*]] = or i44 [[B]], [[C]]
; CHECK-NEXT:    ret i44 [[D]]
;
  %B = shl i44 %A, 33
  %C = lshr i44 %B, 33
  %D = add i44 %B, %C
  ret i44 %D
}

; Fold vector lshr (shl X, C), C -> and X, C' regardless of the number of uses of the shl.

define <2 x i44> @shl_lshr_eq_amt_multi_use_splat_vec(<2 x i44> %A) {
; CHECK-LABEL: @shl_lshr_eq_amt_multi_use_splat_vec(
; CHECK-NEXT:    [[B:%.*]] = shl <2 x i44> [[A:%.*]], <i44 33, i44 33>
; CHECK-NEXT:    [[C:%.*]] = and <2 x i44> [[A]], <i44 2047, i44 2047>
; CHECK-NEXT:    [[D:%.*]] = or <2 x i44> [[B]], [[C]]
; CHECK-NEXT:    ret <2 x i44> [[D]]
;
  %B = shl <2 x i44> %A, <i44 33, i44 33>
  %C = lshr <2 x i44> %B, <i44 33, i44 33>
  %D = add <2 x i44> %B, %C
  ret <2 x i44> %D
}

; Fold shl (lshr X, C), C -> and X, C' regardless of the number of uses of the lshr.

define i43 @lshr_shl_eq_amt_multi_use(i43 %A) {
; CHECK-LABEL: @lshr_shl_eq_amt_multi_use(
; CHECK-NEXT:    [[B:%.*]] = lshr i43 [[A:%.*]], 23
; CHECK-NEXT:    [[C:%.*]] = and i43 [[A]], -8388608
; CHECK-NEXT:    [[D:%.*]] = mul i43 [[B]], [[C]]
; CHECK-NEXT:    ret i43 [[D]]
;
  %B = lshr i43 %A, 23
  %C = shl i43 %B, 23
  %D = mul i43 %B, %C
  ret i43 %D
}

; Fold vector shl (lshr X, C), C -> and X, C' regardless of the number of uses of the lshr.

define <2 x i43> @lshr_shl_eq_amt_multi_use_splat_vec(<2 x i43> %A) {
; CHECK-LABEL: @lshr_shl_eq_amt_multi_use_splat_vec(
; CHECK-NEXT:    [[B:%.*]] = lshr <2 x i43> [[A:%.*]], <i43 23, i43 23>
; CHECK-NEXT:    [[C:%.*]] = and <2 x i43> [[A]], <i43 -8388608, i43 -8388608>
; CHECK-NEXT:    [[D:%.*]] = mul <2 x i43> [[B]], [[C]]
; CHECK-NEXT:    ret <2 x i43> [[D]]
;
  %B = lshr <2 x i43> %A, <i43 23, i43 23>
  %C = shl <2 x i43> %B, <i43 23, i43 23>
  %D = mul <2 x i43> %B, %C
  ret <2 x i43> %D
}

define i37 @test25(i37 %AA, i37 %BB) {
; CHECK-LABEL: @test25(
; CHECK-NEXT:    [[D:%.*]] = and i37 [[AA:%.*]], -131072
; CHECK-NEXT:    [[C2:%.*]] = add i37 [[D]], [[BB:%.*]]
; CHECK-NEXT:    [[F:%.*]] = and i37 [[C2]], -131072
; CHECK-NEXT:    ret i37 [[F]]
;
  %C = lshr i37 %BB, 17
  %D = lshr i37 %AA, 17
  %E = add i37 %D, %C
  %F = shl i37 %E, 17
  ret i37 %F
}

define i40 @test26(i40 %A) {
; CHECK-LABEL: @test26(
; CHECK-NEXT:    [[B:%.*]] = and i40 [[A:%.*]], -2
; CHECK-NEXT:    ret i40 [[B]]
;
  %B = lshr i40 %A, 1
  %C = bitcast i40 %B to i40
  %D = shl i40 %C, 1
  ret i40 %D
}

; OSS-Fuzz #9880
; https://bugs.chromium.org/p/oss-fuzz/issues/detail?id=9880
define i177 @ossfuzz_9880(i177 %X) {
; CHECK-LABEL: @ossfuzz_9880(
; CHECK-NEXT:    [[A:%.*]] = alloca i177, align 8
; CHECK-NEXT:    [[L1:%.*]] = load i177, i177* [[A]], align 8
; CHECK-NEXT:    [[TMP1:%.*]] = icmp eq i177 [[L1]], -1
; CHECK-NEXT:    [[B5_NEG:%.*]] = sext i1 [[TMP1]] to i177
; CHECK-NEXT:    [[B14:%.*]] = add i177 [[L1]], [[B5_NEG]]
; CHECK-NEXT:    [[TMP2:%.*]] = icmp eq i177 [[B14]], -1
; CHECK-NEXT:    [[B1:%.*]] = zext i1 [[TMP2]] to i177
; CHECK-NEXT:    ret i177 [[B1]]
;
  %A = alloca i177
  %L1 = load i177, i177* %A
  %B = or i177 0, -1
  %B5 = udiv i177 %L1, %B
  %B4 = add i177 %B5, %B
  %B2 = add i177 %B, %B4
  %B6 = mul i177 %B5, %B2
  %B20 = shl i177 %L1, %B6
  %B14 = sub i177 %B20, %B5
  %B1 = udiv i177 %B14, %B6
  ret i177 %B1
}
