add_llvm_component_library(LLVMStraightAsmParser
  StraightAsmParser.cpp

  LINK_COMPONENTS
  MC
  MCParser
  StraightDesc
  StraightInfo
  Support

  ADD_TO_COMPONENT
  Straight
  )
