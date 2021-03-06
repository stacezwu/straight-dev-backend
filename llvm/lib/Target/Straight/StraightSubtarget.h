//===-- StraightSubtarget.h - Define Subtarget for the Straight -----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the Straight specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Straight_StraightSUBTARGET_H
#define LLVM_LIB_TARGET_Straight_StraightSUBTARGET_H

#include "StraightFrameLowering.h"
#include "StraightISelLowering.h"
#include "StraightInstrInfo.h"
#include "llvm/CodeGen/SelectionDAGTargetInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"

#define GET_SUBTARGETINFO_HEADER
#include "StraightGenSubtargetInfo.inc"

namespace llvm {
class StringRef;

class StraightSubtarget : public StraightGenSubtargetInfo {
  virtual void anchor();
  StraightInstrInfo InstrInfo;
  StraightFrameLowering FrameLowering;
  StraightTargetLowering TLInfo;
  SelectionDAGTargetInfo TSInfo;

private:
  void initializeEnvironment();
  void initSubtargetFeatures(StringRef CPU, StringRef FS);

protected:

public:
  // This constructor initializes the data members to match that
  // of the specified triple.
  StraightSubtarget(const Triple &TT, const StringRef &CPU, const StringRef &FS,
               const TargetMachine &TM);

  StraightSubtarget &initializeSubtargetDependencies(StringRef CPU, StringRef FS);

  // ParseSubtargetFeatures - Parses features string setting specified
  // subtarget options.  Definition of function is auto generated by tblgen.
  void ParseSubtargetFeatures(StringRef CPU, StringRef TuneCPU, StringRef FS);

  const StraightInstrInfo *getInstrInfo() const override { return &InstrInfo; }
  const StraightFrameLowering *getFrameLowering() const override {
    return &FrameLowering;
  }
  const StraightTargetLowering *getTargetLowering() const override {
    return &TLInfo;
  }
  const SelectionDAGTargetInfo *getSelectionDAGInfo() const override {
    return &TSInfo;
  }
  const TargetRegisterInfo *getRegisterInfo() const override {
    return &InstrInfo.getRegisterInfo();
  }
  // STRAIGHTv3 supports only 64bit (for now)
  MVT getPtrVT() const { return MVT::i64; }
};
} // End llvm namespace

#endif
