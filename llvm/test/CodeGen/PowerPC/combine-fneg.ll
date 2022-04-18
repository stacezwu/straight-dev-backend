; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc < %s -mtriple=powerpc64le-unknown-unknown | FileCheck %s

; Infinite loop identified in D62963.
define <4 x double> @fneg_fdiv_splat(double %a0, <4 x double> %a1) {
; CHECK-LABEL: fneg_fdiv_splat:
; CHECK:       # %bb.0: # %entry
; CHECK-NEXT:    addis 3, 2, .LCPI0_0@toc@ha
; CHECK-NEXT:    # kill: def $f1 killed $f1 def $vsl1
; CHECK-NEXT:    xxspltd 0, 1, 0
; CHECK-NEXT:    addi 3, 3, .LCPI0_0@toc@l
; CHECK-NEXT:    lxvd2x 1, 0, 3
; CHECK-NEXT:    xvredp 2, 0
; CHECK-NEXT:    xxswapd 1, 1
; CHECK-NEXT:    xxlor 3, 1, 1
; CHECK-NEXT:    xvnmsubadp 3, 0, 2
; CHECK-NEXT:    xvmaddadp 2, 2, 3
; CHECK-NEXT:    xvnmsubadp 1, 0, 2
; CHECK-NEXT:    xvnmaddadp 2, 2, 1
; CHECK-NEXT:    xvmuldp 34, 34, 2
; CHECK-NEXT:    xvmuldp 35, 35, 2
; CHECK-NEXT:    blr
entry:
  %splat.splatinsert = insertelement <4 x double> undef, double %a0, i32 0
  %splat.splat = shufflevector <4 x double> %splat.splatinsert, <4 x double> undef, <4 x i32> zeroinitializer
  %div = fdiv reassoc nsz arcp ninf <4 x double> %a1, %splat.splat
  %sub = fsub reassoc nsz <4 x double> <double 0.000000e+00, double 0.000000e+00, double 0.000000e+00, double 0.000000e+00>, %div
  ret <4 x double> %sub
}
