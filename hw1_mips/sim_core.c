/* 046267 Computer Architecture - Spring 2016 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"





/********************data structs*******************/

typedef struct _CONTROL {
	//bool reg_wr; //write or not to reg
	bool reg_dst; //choose between rt & rd fot i-type command
	bool branch; // 
	bool mem_r;
	bool mem_wr;
	bool mem_toReg;
	bool pcSrc;
} CONTROL;

typedef struct _PC {
	bool control_pc;
	uint32_t out;
}PC;


typedef struct _CORE_STATE
{
	PC pc; /// Value of the current program counter (at instruction fetch stage)
	int32_t regFile[SIM_REGFILE_SIZE]; /// Values of each register in the register file
	bool hdu_flag; //flag that shows if a prev command in fetch was lw
	struct
	{
		SIM_cmd cmd;      /// The processed command in each pipe stage
		int32_t src1Val;  /// Actual value of src1 (considering forwarding mux, etc.)
		int32_t src2Val;  /// Actual value of src2 (considering forwarding mux, etc.)
		CONTROL control;  ///values of control
		int dst_value;    // value of returning parameter (example: result of add; destination reg for lw)
	} pipeStageState[SIM_PIPELINE_DEPTH];
} CORE_STATE;

/****************************************************/


/****************global variables*******************/
CORE_STATE mips_state;


/**************************************************/


/***************Function declaration***************/


void ctrl(SIM_cmd_opcode opcode, CONTROL* ctrlr); //function that transmits opcode to control signals
int reset_control(CONTROL* cntr); //reset control
void hdu(); //Hazard detection unit
static void forwarding(); //Forwarding unit
static bool branch(); //return true if there is a branch ( good for deciding which pc is going to be next!!!)
int reset_command(SIM_cmd* command); //resets all commands of state
void cpyCMD(SIM_cmd *src_cmd, SIM_cmd *tar_cmd); //copies commands of one state to another
void shift_pipe(); //moves all commands in mips to the next state

/**************************************************/


void ctrl(SIM_cmd_opcode opcode, CONTROL* ctrlr) {

	ctrlr->reg_dst = (opcode != CMD_NOP) && (opcode != CMD_ADD) && (opcode != CMD_SUB);
	ctrlr->branch = (opcode == CMD_BREQ) || (opcode == CMD_BRNEQ);
	ctrlr->mem_r = (opcode == CMD_LOAD);
	ctrlr->mem_wr = (opcode == CMD_STORE);
	ctrlr->mem_toReg = (opcode == CMD_STORE);
	ctrlr->pcSrc = (opcode == CMD_BREQ) || (opcode == CMD_BRNEQ) || (opcode == CMD_BR);
}


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



/*int writePC(PC* pc, int32_t newPC) { // ????? may be no need
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
*/


void hdu() {
	if (mips_state.pipeStageState[1].cmd.src1 == mips_state.pipeStageState[2].cmd.dst ||
		(mips_state.pipeStageState[1].cmd.isSrc2Imm == false && 
			mips_state.pipeStageState[1].cmd.src2 == mips_state.pipeStageState[2].cmd.dst)) { //nop is needed

		//stall
		mips_state.pc.out -= 4;
		reset_control(&mips_state.pipeStageState[1].control);
	}
		
	else //nop not needed
	{
		return;
	}
}



static void forwarding() {
	if ((mips_state.pipeStageState[3].cmd.opcode == CMD_ADD) && (mips_state.pipeStageState[3].cmd.opcode == CMD_SUB) && (mips_state.pipeStageState[3].cmd.opcode == CMD_LOAD)) {
		if (mips_state.pipeStageState[2].cmd.src1 == mips_state.pipeStageState[3].cmd.dst) //rs_exe vs. rd_mem
			mips_state.pipeStageState[2].src1Val = mips_state.regFile[mips_state.pipeStageState[3].cmd.dst];
		if ((mips_state.pipeStageState[2].cmd.src2 == mips_state.pipeStageState[3].cmd.dst) && (mips_state.pipeStageState[2].cmd.isSrc2Imm == false)) //rt_exe vs. rd_mem
			mips_state.pipeStageState[2].src2Val = mips_state.regFile[mips_state.pipeStageState[3].cmd.dst];
	}
	if ((mips_state.pipeStageState[3].cmd.opcode == CMD_ADD) && (mips_state.pipeStageState[3].cmd.opcode == CMD_SUB) && (mips_state.pipeStageState[3].cmd.opcode == CMD_LOAD) && (mips_state.pipeStageState[3].cmd.dst != mips_state.pipeStageState[4].cmd.dst)) {
		if (mips_state.pipeStageState[2].cmd.src1 == mips_state.pipeStageState[4].cmd.dst) //rs_exe vs. rd_wb
			mips_state.pipeStageState[2].src1Val = mips_state.regFile[mips_state.pipeStageState[4].cmd.dst];
		if (mips_state.pipeStageState[2].cmd.src2 == mips_state.pipeStageState[4].cmd.dst) //rt_exe vs. rd_wb
			mips_state.pipeStageState[2].src2Val = mips_state.regFile[mips_state.pipeStageState[4].cmd.dst];
	}
}



static bool branch() { //return true if there is a branch ( good for deciding which pc is going to be next!!!)
	if (mips_state.pipeStageState[3].control.branch == true) {
		if (mips_state.pipeStageState[3].cmd.opcode == CMD_BR)
			return true;
		if (mips_state.pipeStageState[3].cmd.opcode == CMD_BREQ)
			return (mips_state.pipeStageState[3].src1Val - mips_state.pipeStageState[3].src1Val == 0);
		if (mips_state.pipeStageState[3].cmd.opcode == CMD_BRNEQ)
			return (mips_state.pipeStageState[3].src1Val - mips_state.pipeStageState[3].src1Val != 0);
	}
}


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


/***********Implimentation of API functions*************************/

int SIM_CoreReset(void)
{
	mips_state.pc.out = 0;
	mips_state.pc.control_pc = true;
	mips_state.hdu_flag = false;

	for (int i = 0; i < SIM_REGFILE_SIZE; i++) { //reset registers
		mips_state.regFile[i] = 0;
	}

	for (int i = 0; i < SIM_PIPELINE_DEPTH; i++) { //reset pipeline
		reset_command(&mips_state.pipeStageState[i].cmd);
		reset_control(&mips_state.pipeStageState[i].control);
		mips_state.pipeStageState[i].src1Val = 0;
		mips_state.pipeStageState[i].src2Val = 0;
		mips_state.pipeStageState[i].dst_value = 0;
	}
	return 0;
}

void SIM_CoreClkTick(void)
{
	SIM_cmd fetch_command;


	/**********************FETCH***********************/

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


	/******************DECODE***********************/


	if (mips_state.pipeStageState[1].cmd.opcode != 0) //not nop
	{
		mips_state.pipeStageState[1].src1Val = mips_state.regFile[mips_state.pipeStageState[1].cmd.src1];
		if (mips_state.pipeStageState[1].cmd.isSrc2Imm == false) //src2 is a reg index
		{
			mips_state.pipeStageState[1].src2Val = mips_state.regFile[mips_state.pipeStageState[1].cmd.src2];
		}
		else //src2 is immidiate value
			mips_state.pipeStageState[1].src2Val = mips_state.pipeStageState[1].cmd.src2;

	}


	if (mips_state.hdu_flag == true) //a command in EXE was lw. Activate HDU
	{
		hdu();
	}

	if (mips_state.pipeStageState[1].cmd.opcode == CMD_LOAD) //got lw. need to put flag for further command (hdu)
	{
		mips_state.hdu_flag = true;
	}
	else
		mips_state.hdu_flag = false;

	/******************************EX**************************/

	switch (mips_state.pipeStageState[2].cmd.opcode)
	{
	case CMD_NOP:
		break;
	case CMD_ADD:
		forwarding();
		mips_state.pipeStageState[2].dst_value = mips_state.pipeStageState[2].src1Val + mips_state.pipeStageState[2].src2Val;
		break;
	case CMD_SUB:
		forwarding();
		mips_state.pipeStageState[2].dst_value = mips_state.pipeStageState[2].src1Val - mips_state.pipeStageState[2].src2Val;
		break;
	case CMD_LOAD:
		forwarding();
		mips_state.pipeStageState[2].dst_value = mips_state.pipeStageState[2].src1Val - mips_state.pipeStageState[2].src2Val;
		break;
	case CMD_STORE:
		forwarding();
		mips_state.pipeStageState[2].dst_value = mips_state.pipeStageState[2].src1Val - mips_state.pipeStageState[2].src2Val;
		break;
	case CMD_BR:
		break;
	case CMD_BREQ: //DO we need forward here?? how do we work with branch: what is rs, rt, label??
		break;
	case CMD_BRNEQ: //DO we need forward here??
					
		break;
	}



}

void SIM_CoreGetState(SIM_coreState *curState)
{
	curState->pc = mips_state.pc.out;
	for (int i = 0; i < SIM_REGFILE_SIZE; i++)
	{
		curState->regFile[i] = mips_state.regFile[i];
	}

	for (int i = 0; i < SIM_PIPELINE_DEPTH; i++)
	{
		curState->pipeStageState[i].src1Val = mips_state.pipeStageState[i].src1Val;
		curState->pipeStageState[i].src1Val = mips_state.pipeStageState[i].src2Val;
		cpyCMD(&mips_state.pipeStageState[i].cmd, &curState->pipeStageState[i].cmd);
	}

}