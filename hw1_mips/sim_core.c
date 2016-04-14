/* 046267 Computer Architecture - Spring 2016 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"


typedef enum { ENABLE = 0, DISABLE } STATE;

/********************control**********************/

typedef struct _CONTROL {
	//bool reg_wr; //write or not to reg
	bool reg_dst; //choose between rt & rd fot i-type command
	bool branch; // 
	bool mem_r;
	bool mem_wr;
	bool mem_toReg;
	bool pcSrc;
} CONTROL;

/*function that transmits opcode to control signals*/
void ctrl(SIM_cmd_opcode opcode, CONTROL* ctrlr) {

	ctrlr->reg_dst = (opcode != CMD_NOP) && (opcode != CMD_ADD) && (opcode != CMD_SUB);
	ctrlr->branch = (opcode == CMD_BREQ) || (opcode == CMD_BRNEQ);
	ctrlr->mem_r = (opcode == CMD_LOAD);
	ctrlr->mem_wr = (opcode == CMD_STORE);
	ctrlr->mem_toReg = (opcode == CMD_STORE);
	ctrlr->pcSrc = (opcode == CMD_BREQ) || (opcode == CMD_BRNEQ) || (opcode == CMD_BR);
}

/*reset control*/
int reset_control(CONTROL* cntr) {
	if (cntr == NULL) return -1;
	cntr->branch = 0;
	cntr->mem_r = 0;
	cntr->mem_toReg = 0;
	cntr->mem_wr = 0;
	cntr->reg_dst = 0;
	cntr->pcSrc = 0;
	return 0;
}

/**********************************************************/


/************************Registers*************************/
typedef struct _REG_FILE {
	bool control_write; 
	int32_t regFile[SIM_REGFILE_SIZE]; // Values of each register in the register file
	int32_t read_data1, read_data2;
}REG_FILE;

//////*****************************************************************************change functions
/*Gets register adress and returns value of the register*/
void read_register(REG_FILE* reg, int reg1, int reg2) {
	reg->read_data1 = reg->regFile[reg1];
	reg->read_data2 = reg->regFile[reg2];
	return;
}

void write_register(REG_FILE* reg, int write_register, int32_t write_date) {
	if (reg->control_write) reg->regFile[write_register] = write_date;
	return;
}

/**********************************************************/


/**********************PC**********************************/

typedef struct _PC {
	bool control_pc;
	uint32_t out;
} PC;

int writePC(PC* pc, int32_t newPC) { // ????? may be no need
	if (pc == NULL) return -1;
	if (pc->control_pc == true) {
		pc->out = newPC;
		return 0;
	}
	else
	{
		return 0; //contorl is disabled. don't do any change
	}
}

/**********************************************************/




/**************Hazard detection unit**********************/




/*********************************************************/

/*******************Forwarding unit***********************/

static void forwarding(CORE_STATE* cst) {
	if (cst->pipeStageState[3].cmd.opcode != 0) {
		if (cst->pipeStageState[2].cmd.src1 == cst->pipeStageState[3].cmd.dst) //rs_exe vs. rd_mem
			cst->pipeStageState[2].src1Val = cst->regFile[cst->pipeStageState[3].cmd.dst];
		if (cst->pipeStageState[2].cmd.src2 == cst->pipeStageState[3].cmd.dst) //rt_exe vs. rd_mem
			cst->pipeStageState[2].src2Val = cst->regFile[cst->pipeStageState[3].cmd.dst];
	}
	if ((cst->pipeStageState[4].cmd.opcode != 0) && (cst->pipeStageState[3].cmd.dst != cst->pipeStageState[4].cmd.dst)) {
		if (cst->pipeStageState[2].cmd.src1 == cst->pipeStageState[4].cmd.dst) //rs_exe vs. rd_wb
			cst->pipeStageState[2].src1Val = cst->regFile[cst->pipeStageState[4].cmd.dst];
		if (cst->pipeStageState[2].cmd.src2 == cst->pipeStageState[4].cmd.dst) //rt_exe vs. rd_wb
			cst->pipeStageState[2].src2Val = cst->regFile[cst->pipeStageState[4].cmd.dst];
	}
}



static bool brunch(CORE_STATE* cst) {
	if (cst->pipeStageState[3].control.branch == true) {
		if (cst->pipeStageState[3].cmd.opcode == CMD_BR)
			return true;
		if (cst->pipeStageState[3].cmd.opcode == CMD_BREQ)
			return (cst->pipeStageState[3].src1Val - cst->pipeStageState[3].src1Val == 0);
		if (cst->pipeStageState[3].cmd.opcode == CMD_BRNEQ)
			return (cst->pipeStageState[3].src1Val - cst->pipeStageState[3].src1Val != 0);
	}
}




typedef struct _CORE_STATE
{
	PC pc; /// Value of the current program counter (at instruction fetch stage)
	REG_FILE regFile; /// Values of each register in the register file
	bool hdu_flag; //flag that shows if a prev command in fetch was lw
	struct
	{
		SIM_cmd cmd;      /// The processed command in each pipe stage
		int32_t src1Val;  /// Actual value of src1 (considering forwarding mux, etc.)
		int32_t src2Val;  /// Actual value of src2 (considering forwarding mux, etc.)
		CONTROL control;  ///values of control
	} pipeStageState[SIM_PIPELINE_DEPTH];
} CORE_STATE;

/****************global variables*******************/
CORE_STATE mips_state;


/**************************************************/



/******************additional functions for implimentation*******************/
int reset_command(SIM_cmd* command) {
	if (command == NULL) return -1;
	command->dst = 0;
	command->isSrc2Imm = false;
	command->opcode = 0;
	command->src1 = 0;
	command->src2 = 0;
}

void cpyCMD(SIM_cmd *src_cmd, SIM_cmd *tar_cmd) {
	tar_cmd->opcode = src_cmd->opcode;
	tar_cmd->src1 = src_cmd->src1;
	tar_cmd->src2 = src_cmd->src2;
	tar_cmd->isSrc2Imm = src_cmd->isSrc2Imm;
	tar_cmd->dst = src_cmd->dst;
}

void shift_pipe() {

	for (int i = SIM_PIPELINE_DEPTH - 1; i > 0; i--) {
		mips_state.pipeStageState[i] = mips_state.pipeStageState[i - 1];
	}
}
/*********************************************************/


/***********Implimentation of API functions*************************/

int SIM_CoreReset(void)
{
	mips_state.pc.out = 0;
	mips_state.pc.control_pc = true;
	mips_state.hdu_flag = false;

	mips_state.regFile.read_data1 = 0;
	mips_state.regFile.read_data2 = 0;
	mips_state.regFile.control_write = false;
	for (int i = 0; i < SIM_REGFILE_SIZE; i++) { //reset registers
		mips_state.regFile.regFile[i] = 0;
	}

	for (int i = 0; i < SIM_PIPELINE_DEPTH; i++) { //reset pipeline
		reset_command(&mips_state.pipeStageState[i].cmd);
		reset_control(&mips_state.pipeStageState[i].control);
		mips_state.pipeStageState[i].src1Val = 0;
		mips_state.pipeStageState[i].src2Val = 0;
	}
	return 0;
}

void SIM_CoreClkTick(void)
{
	SIM_cmd fetch_command;


	/*FETCH*/

	shift_pipe();
	SIM_MemInstRead(mips_state.pc.out, &fetch_command);
	if ((fetch_command.opcode > CMD_MAX) || (fetch_command.opcode < 0)) //check validaty of opcode
	{
		exit(-1);
	}

	mips_state.pipeStageState[0].cmd = fetch_command;
	ctrl(fetch_command.opcode, &mips_state.pipeStageState[0].control); //transmit opcode to command signals

	mips_state.pc.control_pc = true;
	mips_state.pc.out += 4; //pc+4

	//decide which PC will be next

	/*DECODE*/


	if (mips_state.pipeStageState[1].cmd.opcode != 0) //not nope
	{
		//TODO check copy of from reg file 
		//mips_state.pipeStageState[1].src1Val = mips_state.regFile[mips.pipeStageState[1].cmd.src1];
		if (mips_state.pipeStageState[1].cmd.isSrc2Imm == false)
		{
			read_register(&mips_state.regFile, mips_state.pipeStageState[1].cmd.src1, mips_state.pipeStageState[1].cmd.src2);
			mips_state.pipeStageState[1].src1Val = mips_state.pipeStageState[1].cmd.src1;
			mips_state.pipeStageState[1].src2Val = mips_state.pipeStageState[1].cmd.src2;
		}
		else
		{
			read_register(&mips_state.regFile, mips_state.pipeStageState[1].cmd.src1, mips_state.pipeStageState[1].cmd.src2);
			mips_state.pipeStageState[1].src2Val = mips_state.pipeStageState[1].cmd.src2;
		}


	}
	
	if (mips_state.pipeStageState[1].cmd.opcode == CMD_LOAD) //got lw. need put flag for further command (hdu)
	{
		mips_state.hdu_flag = true;
	}
	else
		mips_state.hdu_flag = false;

	if (mips_state.hdu_flag == true) //check if there is data dependecy of prev. lw
	{

	}





}

void SIM_CoreGetState(SIM_coreState *curState)
{
	curState->pc = mips_state.pc.out;
	for (int i = 0; i < SIM_REGFILE_SIZE; i++)
	{
		curState->regFile[i] = mips_state.regFile.regFile[i];
	}

	for (int i = 0; i < SIM_PIPELINE_DEPTH; i++)
	{
		curState->pipeStageState[i].src1Val = mips_state.pipeStageState[i].src1Val;
		curState->pipeStageState[i].src1Val = mips_state.pipeStageState[i].src2Val;
		cpyCMD(&mips_state.pipeStageState[i].cmd, &curState->pipeStageState[i].cmd);
	}

}