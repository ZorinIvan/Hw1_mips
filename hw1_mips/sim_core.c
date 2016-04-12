/* 046267 Computer Architecture - Spring 2016 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"

/*control data struct*/
typedef struct _CONTROL{
	SIM_cmd_opcode opcode;
	bool reg_wr; //write or not to reg
	bool reg_dst; //choose between rt & rd fot i-type command
	bool branch; // 
	//bool ALUSrc; 
	//bool PCSrc;
	bool mem_r;
	bool mem_wr;
	bool mem_toReg;
} CONTROL;

/*Registers*/
typedef struct _REG_FILE {
	int32_t regFile[SIM_REGFILE_SIZE]; /// Values of each register in the register file
	int32_t read_data1, read_data2;
}REG_FILE;



/*register between fetch and decode*/
typedef struct _IF_DEC_REG {
	/*input*/
	int32_t inst_in;
	int pc_plus4;

	/*output*/
	int rs_out, rt_out, rd_out;
	SIM_cmd_opcode opcode_out;
}IF_DEC_REG;


/*register between decode and execute*/
typedef struct _DEC_EXE_REG {
	/*input*/
	int32_t regRS_in, regRT_in;

	/*output*/
	int rs_in_out, rd_in_out, rt_in_out;

	/*control*/
	CONTROL contorl;
}DEC_EXE_REG;

typedef struct _EXE_MEM_REG {
	int32_t ALU_RES_in;
	int32_t ALU_RES_out;
	int MUX_RT_RD_in;
	int MUX_RT_RD_out;
}EXE_MEM_REG;

typedef struct _MEM_WB_REG {
	int32_t ALU_RES_in;
	int32_t ALU_RES_out;
	int32_t MEM_in_out;
	int MUX_RT_RD_in;
	int MUX_RT_RD_out;
}MEM_WB_REG;

typedef struct _REGFILE {
	int32_t regNum[SIM_REGFILE_SIZE];
	int32_t rs;
	int32_t rt;
}REGFILE;



int SIM_CoreReset(void)
{

	return -1;
}

void SIM_CoreClkTick(void)
{
}

void SIM_CoreGetState(SIM_coreState *curState)
{
}
