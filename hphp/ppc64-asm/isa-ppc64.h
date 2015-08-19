/**
 * Copyright 2015, IBM
 * All Rights Reserved
 *
 * @author: Rogerio Alves
 *
 * This is a experimental macro assembler for PPC64.
 * Don't expect to find all instructions here.
 *
 * If you're looking for something more fully baked, here are some options
 * to consider use Nanojit or LLVM, both of which translate abstract virtual 
 * machine instructions to the native target architecture.
 *
 */

#ifndef INCLUDE_PPC64_ISA_H_
#define INCLUDE_PPC64_ISA_H_

#include <cstdint>

namespace ppc64_asm {

typedef uint32_t PPC64Instr;

/**
 * Instruction Format encoders
 */
typedef union XO_format {
  struct {
    uint32_t Rc:1;
    uint32_t XO:9;
    uint32_t OE:1;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XO_form_t;

typedef union X_format {
  struct {
    uint32_t Rc:1;
    uint32_t XO:10;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} X_form_t;

typedef union D_format {
  struct {
    uint32_t D:16;
    uint32_t RA:5;
    uint32_t RT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} D_form_t;

typedef union I_format {
  struct {
    uint32_t LK:1;
    uint32_t AA:1;
    uint32_t LI:24;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} I_form_t;

typedef union B_format {
  struct {
    uint32_t LK:1;
    uint32_t AA:1;
    uint32_t BD:14;
    uint32_t BI:5;
    uint32_t BO:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} B_form_t;

typedef union SC_format {
  struct {
    uint32_t B31:1;
    uint32_t B30:1;
    uint32_t LEV:7;
    uint32_t RSV3:4;
    uint32_t RSV2:5;
    uint32_t RSV1:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} SC_form_t;

typedef union DS_format {
  struct {
    uint32_t XO:2;
    uint32_t DS:19;
    uint32_t RA:5;
    uint32_t RT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} DS_form_t;

typedef union DQ_format {
  struct {
    uint32_t RSV:4;
    uint32_t DQ:17;
    uint32_t RA:5;
    uint32_t RTp:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} DQ_form_t;

typedef union instruction {
  struct {
    uint32_t LK:1;
    uint32_t XO:10;
    uint32_t BB:5;
    uint32_t BA:5;
    uint32_t BT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XL_form_t;

typedef union XFX_format {
  struct {
    uint32_t RSV:1;
    uint32_t XO:10;
    uint32_t SPRlo:5;  // see note 1: the order of the two 5-bit halves
    uint32_t SPRhi:5;  // of the SPR number is reversed
    uint32_t RT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XFX_form_t;

typedef union XFL_format {
  struct {
    uint32_t Rc:1;
    uint32_t XO:10;
    uint32_t FRB:5;
    uint32_t W:1;
    uint32_t FLM:8;
    uint32_t L:1;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XFL_form_t;

typedef union XX1_format {
  struct {
    uint32_t TX:1;
    uint32_t XO:10;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t T:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XX1_form_t;

typedef union XX2_format {
  struct {
    //TODO
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XX2_form_t;

typedef union XX3_format {
  struct {
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XX3_form_t;

typedef union XX4_format {
  struct {
    uint32_t TX:1;
    uint32_t BX:1;
    uint32_t AX:1;
    uint32_t CX:1;
    uint32_t XO:2;
    uint32_t C:5;
    uint32_t B:5;
    uint32_t A:5;
    uint32_t T:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XX4_form_t;

typedef union XS_format {
  struct {
    uint32_t Rc:1;
    uint32_t sh:1;
    uint32_t XO:9;
    uint32_t SH:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} XS_form_t;

typedef union A_format {
  struct {
    uint32_t Rc:1;
    uint32_t XO:5;
    uint32_t FRC:5;
    uint32_t FRB:5;
    uint32_t FRA:5;
    uint32_t FRT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} A_form_t;

typedef union M_format {
  struct {
    uint32_t Rc:1;
    uint32_t ME:5;
    uint32_t MB:5;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} M_form_t;

typedef union MD_format {
  struct {
    uint32_t Rc:1;
    uint32_t sh:1;
    uint32_t XO:3;
    uint32_t MB:6;
    uint32_t SH:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} MD_form_t;

typedef union MDS_format {
  struct {
    uint32_t Rc:1;
    uint32_t XO:4;
    uint32_t MB:6;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} MDS_form_t;

typedef union VA_format {
  struct {
    uint32_t XO:6;
    uint32_t VRC:5;
    uint32_t VRB:5;
    uint32_t VRA:5;
    uint32_t VRT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} VA_form_t;

typedef union VC_format {
  struct {
    uint32_t XO:10;
    uint32_t Rc:1;
    uint32_t VRB:5;
    uint32_t VRA:5;
    uint32_t VRT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} VC_form_t;

typedef union VX_format {
  struct {
    uint32_t XO:11;
    uint32_t VRB:5;
    uint32_t VRA:5;
    uint32_t VRT:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} VX_form_t;

typedef union EVX_format {
  struct {
    uint32_t XO:11;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} EVX_form_t;

typedef union EVS_format {
  struct {
    uint32_t BFA:3;
    uint32_t XO:8;
    uint32_t RB:5;
    uint32_t RA:5;
    uint32_t RS:5;
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} EVS_form_t;

typedef union Z22_format {
  struct {
    //TODO:
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} Z22_form_t;

typedef union Z23_format {
  struct {
    //TODO:
    uint32_t OP:6;
  };
  PPC64Instr instruction;
} Z23_form_t;

} // namespace ppc64_asm

#endif
