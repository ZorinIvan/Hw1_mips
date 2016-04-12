/* 046267 Computer Architecture - Spring 2016 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"


typedef enum {ENABLE=0, DISABLE} STATE;

/********************control**********************/

typedef struct _CONTROL {
	//bool reg_wr; //write or not to reg
	bool reg_dst; //choose between rt & rd fot i-type command
	bool branch; // 
	bool mem_r;
	bool mem_wr;
	bool mem_toReg;
} CONTROL;

/*function that transmits opcode to control signals*/
void ctrl(SIM_cmd_opcode opcode, CONTROL* ctrlr) {

	ctrlr->reg_dst = (opcode != CMD_NOP) && (opcode != CMD_ADD) && (opcode != CMD_SUB);
	ctrlr->branch = (opcode == CMD_BREQ) || (opcode == CMD_BRNEQ);
	ctrlr->mem_r = (opcode == CMD_LOAD);
	ctrlr->mem_wr = (opcode == CMD_STORE);
	ctrlr->mem_toReg = (opcode == CMD_STORE);
}

/*reset control*/
int reset_control(CONTROL* cntr) {
	if (cntr == NULL) return -1;
	cntr->branch = 0;
	cntr->mem_r = 0;
	cntr->mem_toReg = 0;
	cntr->mem_wr = 0;
	cntr->reg_dst = 0;
	return 0;
}

/**********************************************************/


/************************Registers*************************/
typedef struct _REG_FILE {
	bool enable; //?????
	int32_t regFile[SIM_REGFILE_SIZE]; // Values of each register in the register file
	int32_t read_data1, read_data2;
}REG_FILE;

//////*****************************************************************************change functions
/*Gets register adress and returns value of the register*/
void read_register(REG_FILE* reg, int reg1, int reg2) {
	reg->read_data1 = reg->regFile[reg1];
	reg->read_data2 = reg->regFile[reg2];
}

void write_register(REG_FILE* reg, int write_register, int32_t write_date) {
	if (reg->enable) reg->regFile[write_register] = write_date;
}

/**********************************************************/


/******************reset and copy command*******************/
int reset_command(SIM_cmd* command) {
	if (command == NULL) return -1;
	command->dst = 0;
	command->isSrc2Imm = false;
	command->opcode=0;
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

/*****************************************************/


typedef struct _CORE_STATE
{
	int32_t pc; /// Value of the current program counter (at instruction fetch stage)
	int32_t regFile[SIM_REGFILE_SIZE]; /// Values of each register in the register file
	struct
	{
		SIM_cmd cmd;      /// The processed command in each pipe stage
		int32_t src1Val;  /// Actual value of src1 (considering forwarding mux, etc.)
		int32_t src2Val;  /// Actual value of src2 (considering forwarding mux, etc.)
		CONTROL control;  ///values of control
	} pipeStageState[SIM_PIPELINE_DEPTH];
} CORE_STATE;



/****************global variables*******************/
CORE_STATE current_mips_state;


/**************************************************/





int SIM_CoreReset(void)
{
	current_mips_state.pc = 0;

	for (int i = 0; i < SIM_REGFILE_SIZE; i++) { //reset pipeline
		current_mips_state.regFile[i] = 0;
	}

	for (int i = 0; i < SIM_PIPELINE_DEPTH; i++) { //reset state
		reset_command(&current_mips_state.pipeStageState[i].cmd);
		reset_control(&current_mips_state.pipeStageState[i].control);
		current_mips_state.pipeStageState[i].src1Val = 0;
		current_mips_state.pipeStageState[i].src2Val = 0;
	}
	return 0;
}

void SIM_CoreClkTick(void)
{
}

void SIM_CoreGetState(SIM_coreState *curState)
{
	curState->pc = current_mips_state.pc;
	for (int i = 0; i < SIM_REGFILE_SIZE; i++)
	{
		curState->regFile[i] = current_mips_state.regFile[i];
	}

	for (int i = 0; i < SIM_PIPELINE_DEPTH; i++)
	{
		curState->pipeStageState[i].src1Val = current_mips_state.pipeStageState[i].src1Val;
		curState->pipeStageState[i].src1Val = current_mips_state.pipeStageState[i].src2Val;
		cpyCMD(&current_mips_state.pipeStageState[i].cmd, &curState->pipeStageState[i].cmd);
	}

}