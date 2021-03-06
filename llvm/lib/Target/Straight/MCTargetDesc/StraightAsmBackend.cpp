//===-- StraightAsmBackend.cpp - Straight Assembler Backend -------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/StraightMCTargetDesc.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/Support/EndianStream.h"
#include <cassert>
#include <cstdint>

using namespace llvm;

namespace {

class StraightAsmBackend : public MCAsmBackend {
public:
  StraightAsmBackend(support::endianness Endian) : MCAsmBackend(Endian) {}
  ~StraightAsmBackend() override = default;

  void applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                  const MCValue &Target, MutableArrayRef<char> Data,
                  uint64_t Value, bool IsResolved, const MCSubtargetInfo *STI) const override;

  std::unique_ptr<MCObjectTargetWriter>
  createObjectTargetWriter() const override;

  // No instruction requires relaxation
  bool fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                            const MCRelaxableFragment *DF,
                            const MCAsmLayout &Layout) const override {
    return false;
  }

  unsigned getNumFixupKinds() const override { return 1; }

  bool mayNeedRelaxation(const MCInst &Inst, const MCSubtargetInfo &STI) const override { return false; }

  void relaxInstruction(MCInst &Inst, const MCSubtargetInfo &STI) const override {}

  bool writeNopData(raw_ostream &OS, uint64_t Count) const override;
};

} // end anonymous namespace

bool StraightAsmBackend::writeNopData(raw_ostream &OS, uint64_t Count) const {
  if ((Count % 8) != 0)
    return false;

  for (uint64_t i = 0; i < Count; i += 8)
    support::endian::write<uint64_t>(OS, 0x15000000, Endian);

  return true;
}

void StraightAsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                               const MCValue &Target,
                               MutableArrayRef<char> Data, uint64_t Value,
                               bool IsResolved, const MCSubtargetInfo* STI) const {
  if (Fixup.getKind() == FK_SecRel_4 || Fixup.getKind() == FK_SecRel_8) {
    assert(Value == 0);
  } else if (Fixup.getKind() == FK_Data_4) {
    support::endian::write<uint32_t>(&Data[Fixup.getOffset()], Value, Endian);
  } else if (Fixup.getKind() == FK_Data_8) {
    support::endian::write<uint64_t>(&Data[Fixup.getOffset()], Value, Endian);
  } else if (Fixup.getKind() == FK_PCRel_4) {
    Value = (uint32_t)((Value - 8) / 8);
    if (Endian == support::little) {
      Data[Fixup.getOffset() + 1] = 0x10;
      support::endian::write32le(&Data[Fixup.getOffset() + 4], Value);
    } else {
      Data[Fixup.getOffset() + 1] = 0x1;
      support::endian::write32be(&Data[Fixup.getOffset() + 4], Value);
    }
  } else {
    assert(Fixup.getKind() == FK_PCRel_2);
    Value = (uint16_t)((Value - 8) / 8);
    support::endian::write<uint16_t>(&Data[Fixup.getOffset() + 2], Value,
                                     Endian);
  }
}

std::unique_ptr<MCObjectTargetWriter>
StraightAsmBackend::createObjectTargetWriter() const {
  return createStraightELFObjectWriter(0);
}

MCAsmBackend *llvm::createStraightAsmBackend(const Target &T,
                                        const MCSubtargetInfo &STI,
                                        const MCRegisterInfo &MRI,
                                        const MCTargetOptions &) {
  return new StraightAsmBackend(support::little);
}

MCAsmBackend *llvm::createStraightbeAsmBackend(const Target &T,
                                          const MCSubtargetInfo &STI,
                                          const MCRegisterInfo &MRI,
                                          const MCTargetOptions &) {
  return new StraightAsmBackend(support::big);
}
