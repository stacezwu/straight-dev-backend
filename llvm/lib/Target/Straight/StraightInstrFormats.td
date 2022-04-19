//===-- StraightInstrFormats.td - Straight Instruction Formats -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//


class InstructionFormat<bits<5> operandnum> {}

def ExactForm      : InstructionFormat<0>; 
def Imm20Form      : InstructionFormat<1>; // Imm20-funct2-funct3-Opcode7
def FenceForm      : InstructionFormat<2>; // 7{0}-4{0}-succ4-pred4-funct3-funct3-Opcode7
def OtherIntForm   : InstructionFormat<3>; // 7bit-12bit-funct3-funct3-Opcode7
def RegImmForm     : InstructionFormat<4>; // Reg7-Imm12-funct2-funct3-Width1-Opcode7
def RegRegForm     : InstructionFormat<5>; // Reg7-Reg7-Ex5-funct2-funct3-Width1-Opcode7
def RegRegImmForm  : InstructionFormat<6>; // Reg7-Reg7-Imm12-Opcode6
def AtomicForm     : InstructionFormat<7>; // Reg7-Reg7-aq1-rl1-funct3-funct2-funct3-Width1-Opcode7
//def FMAForm        : InstructionFormat<8>; // Reg7-Reg7-Reg7-RM3-Width1-Opcode7
def FloatForm      : InstructionFormat<9>; // Reg7-Reg7-funct5-funct2-RM3-Width1-Opcode7
def RegShamt5Form  : InstructionFormat<10>; // Reg7-2{0}-Shamt5-Ex5-funct2-funct3-{0}-Opcode7
def RegShamt6Form  : InstructionFormat<11>; // Reg7-{0}-Shamt6-Ex5-funct2-funct3-{1}-Opcode7

class StraightInst<dag outs, dag ins, string asmstr, InstructionFormat instformat> : Instruction {
  InstructionFormat InstFormat = instformat;

  field bits<64> Inst;

  let Size = 8;
  let Namespace = "Straight";
  let DecoderNamespace = "Straight";
  let OutOperandList = outs;
  let InOperandList = ins;
  let AsmString = asmstr;
  let Pattern = [];

  field bits<64> SoftFail = 0;
}

class StraightInstExactForm<bits<64> opA, dag outs, dag ins, string asmstr, list<dag> pattern>
  : StraightInst<outs, ins, asmstr, ExactForm> {
  let Inst = opA;
  let Pattern = pattern;
}

class StraightInstImm52Form<bits<7> opA, bits<3> opB, bits<2> opC, dag outs, dag ins, string asmstr, string argstr>
  : StraightInst<outs, ins, asmstr, Imm20Form> {
  bits<52> Imm52;

  let Inst{6-0} = opA;
  let Inst{9-7} = opB;
  let Inst{11-10} = opC;
  let Inst{63-12} = Imm52;
}

class StraightInstOtherIntForm<bits<7> opA, bits<3> opB, bits<3> opC, dag outs, dag ins, string asmstr, string argstr>
  : StraightInst<outs, ins, asmstr, OtherIntForm> {
  bits<28> Imm28;
  bits<23> ImmOrReg23;

  let Inst{6-0} = opA;
  let Inst{9-7} = opB;
  let Inst{12-10} = opC;
  let Inst{40-13} = Imm28;
  let Inst{63-41} = ImmOrReg23;
}

class StraightInstFenceForm<bits<7> opA, bits<3> opB, bits<3> opC, dag outs, dag ins, string asmstr, string argstr>
  : StraightInst<outs, ins, asmstr, FenceForm> {
  bits<4> Succ;
  bits<4> Pred;

  let Inst{6-0} = opA;
  let Inst{9-7} = opB;
  let Inst{12-10} = opC;
  let Inst{16-13} = Pred;
  let Inst{20-17} = Succ;
  let Inst{31-21} = 0;
}

class StraightInstRegImmForm<bits<7> opA, bit opW, bits<3> opX, bits<2> opY, dag outs, dag ins, string asmstr, string argstr>
  : StraightInst<outs, ins, asmstr, RegImmForm> {
  bits<28> Imm28;
  bits<23> Reg23;

  let Inst{6-0} = opA;
  let Inst{7} = opW;
  let Inst{10-8} = opX;
  let Inst{12-11} = opY;
  let Inst{40-13} = Imm28;
  let Inst{63-41} = Reg23;
}

class StraightInstRegRegForm<bits<7> opA, bit opW, bits<3> opX, bits<2> opY, bits<5> opZ, dag outs, dag ins, string asmstr, string argstr>
  : StraightInst<outs, ins, asmstr, RegRegForm> {
  bits<23> Reg23B;
  bits<23> Reg23A;

  let Inst{6-0} = opA;
  let Inst{7} = opW;
  let Inst{10-8} = opX;
  let Inst{12-11} = opY;
  let Inst{17-13} = opZ;
  let Inst{40-18} = Reg23B;
  let Inst{63-41} = Reg23A;
}

class StraightInstRegRegImmForm<bits<6> opA, dag outs, dag ins, string asmstr, string argstr>
  : StraightInst<outs, ins, asmstr, RegRegImmForm> {
  bits<12> Imm12;
  bits<23> Reg23B;
  bits<23> Reg23A;

  let Inst{5-0} = opA;
  let Inst{17-6} = Imm12;
  let Inst{40-18} = Reg23B;
  let Inst{63-41} = Reg23A;
}


class StraightInstAtomicForm<bits<7> opA, bit opW, bits<3> opX, bits<2> opY, bits<3> opZ, bit rl, bit aq, dag outs, dag ins, string asmstr, string argstr>
  : StraightInst<outs, ins, asmstr, AtomicForm> {
  bits<23> Reg23B;
  bits<23> Reg23A;

  let Inst{6-0} = opA;
  let Inst{7} = opW;
  let Inst{10-8} = opX;
  let Inst{12-11} = opY;
  let Inst{15-13} = opZ;
  let Inst{16} = rl;
  let Inst{17} = aq;
  let Inst{40-18} = Reg23B;
  let Inst{63-41} = Reg23A;
}

//FIXME: not sure how to divide up 3 registers
// class StraightInstFMAForm<bits<7> opA, bit opW, bits<3> opX, dag outs, dag ins, string asmstr, string argstr>
//   : StraightInst<outs, ins, asmstr, FMAForm> {
//   bits<7> Reg7C;
//   bits<7> Reg7B;
//   bits<7> Reg7A;

//   let Inst{6-0} = opA;
//   let Inst{7} = opW;
//   let Inst{10-8} = opX;
//   let Inst{17-11} = Reg7C;
//   let Inst{24-18} = Reg7B;
//   let Inst{31-25} = Reg7A;
// }

class StraightInstFloatForm<bits<7> opA, bit opW, bits<3> opX, bits<2> opY, bits<5> opZ, dag outs, dag ins, string asmstr, string argstr>
  : StraightInst<outs, ins, asmstr, FloatForm> {
  bits<23> Reg23B;
  bits<23> Reg23A;

  let Inst{6-0} = opA;
  let Inst{7} = opW;
  let Inst{10-8} = opX;
  let Inst{12-11} = opY;
  let Inst{17-13} = opZ;
  let Inst{40-18} = Reg23B;
  let Inst{63-41} = Reg23A;
}

class StraightInstRegShamt5Form<bits<7> opA, bit opW, bits<3> opX, bits<2> opY, bits<5> opZ, dag outs, dag ins, string asmstr, string argstr>
  : StraightInst<outs, ins, asmstr, RegShamt5Form> {
  bits<5> Shamt5;
  bits<39> Reg39;

  let Inst{6-0} = opA;
  let Inst{7} = opW;
  let Inst{10-8} = opX;
  let Inst{12-11} = opY;
  let Inst{17-13} = opZ;
  let Inst{22-18} = Shamt5;
  let Inst{24-23} = 0;
  let Inst{63-25} = Reg39;
}

class StraightInstRegShamt6Form<bits<7> opA, bit opW, bits<3> opX, bits<2> opY, bits<5> opZ, dag outs, dag ins, string asmstr, string argstr>
  : StraightInst<outs, ins, asmstr, RegShamt6Form> {
  bits<6> Shamt6;
  bits<39> Reg39;

  let Inst{6-0} = opA;
  let Inst{7} = opW;
  let Inst{10-8} = opX;
  let Inst{12-11} = opY;
  let Inst{17-13} = opZ;
  let Inst{23-18} = Shamt6;
  let Inst{24} = 0;
  let Inst{63-25} = Reg39;
}
