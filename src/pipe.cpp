/*
 * Computer Architecture CSE530
 * MIPS pipeline cycle-accurate simulator
 * PSU
 */

#include "pipe.h"
#include "mips.h"
#include "abstract_memory.h"
#include "static_nt_branch_predictor.h"
#include <cstdio>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cassert>
#include "util.h"

#include "ooo_data_structures.h"

#define OP_DEC_DONE(op) (op->inst_decoded_done[0] && op->inst_decoded_done[1] && op->inst_decoded_done[2] && op->inst_decoded_done[3]) 
#define OP_COMPL(op) \
do {\
    /* Complete here: write the value to the physical register */ \
    if (op->reg_phy_dst != -1) {\
        mapTable.ready[op->reg_phy_dst] = true; \
        mapTable.regValue[op->reg_phy_dst] = op->reg_phy_dst_value; \
        /* Update the value to RS */ \
        updateRS(reservStation, op->reg_phy_dst); \
        /* Update the completion flag */ \
        op->compl_done = true; \
    }\
} while(0)

static MapTable mapTable;
static FreeList freeList(64);
static RS reservStation;
static ROB reorderedBuffer(8);
static LSQ ldstQueue(4);
static ArchMap archMap;
static std::set<Pipe_Op*> execSet; //execution list
static std::set<Pipe_Op*> complSet; //completion list

#if 1
#define DEBUG(msg,op,i) \
do{ \
    printf(msg);\
    printOp2(op, i);\
} while(0)

#define DEBUG_MSG(fmt,...) fprintf(stderr, fmt, __VA_ARGS__)

#endif

/* debug */
void printOp(Pipe_Op *op) {
	if (op)
		printf(
				"OP (PC=%08x inst=%08x) src1=R%d (%08x) src2=R%d (%08x) dst=R%d valid %d (%08x) br=%d taken=%d dest=%08x mem=%d addr=%08x\n",
				op->pc, op->instruction, op->reg_src1, op->reg_src1_value,
				op->reg_src2, op->reg_src2_value, op->reg_dst,
				op->reg_dst_value_ready, op->reg_dst_value, op->is_branch,
				op->branch_taken, op->branch_dest, op->is_mem, op->mem_addr);
	else
		printf("(null)\n");
}

void printOp2(Pipe_Op *op, int i) {
	if (op)
		printf(
				"OP (PC=%08x inst=%08x) src1=R%d(Phy R%d) src2=R%d (Phy R%d) "
                "dst=R%d (Phy R%d) br=%d taken=%d dest=%08x mem=%d addr=%08x\n",
				op->pc+i*4, op->instruction[i], 
                op->reg_src1, op->reg_phy_src1,
				op->reg_src2, op->reg_phy_src2,
                op->reg_dst, op->reg_phy_dst,
				op->is_branch,
				op->branch_taken, op->branch_dest, op->is_mem, op->mem_addr);
	else
		printf("(null)\n");
}

PipeState::PipeState() :
		fetch_op(nullptr), decode_op(nullptr), execute_op(nullptr), mem_op(
				nullptr), wb_op(nullptr), data_mem(nullptr), inst_mem(nullptr), HI(
				0), LO(0), branch_recover(0), branch_dest(0), branch_flush(0), RUN_BIT(
				true), stat_cycles(0), stat_inst_retire(0), stat_inst_fetch(0), stat_squash(
				0) {
	//initialize the register file
	for (int i = 0; i < 32; i++) {
		REGS[i] = 0;
	}
	//initialize PC
	PC = 0x00400000;
	//initialize the branch predictor
	BP = new StaticNTBranchPredictor();
}

PipeState::~PipeState() {
	if (fetch_op)
		free(fetch_op);
	if (decode_op)
		free(decode_op);
	if (execute_op)
		free(execute_op);
	if (mem_op)
		free(mem_op);
	if (wb_op)
		free(wb_op);
	delete BP;
}

void PipeState::pipeCycle() {
	if (DEBUG_PIPE) {
		printf("\n\n----\nCycle : %lu\nPIPELINE:\n", currCycle);
		printf("DECODE: ");
		printOp(decode_op);
		printf("EXEC : ");
		printOp(execute_op);
		printf("MEM  : ");
		printOp(mem_op);
		printf("WB   : ");
		printOp(wb_op);
		printf("\n");
	}
    printf("\n\n----\nCycle : %lu\nPIPELINE:\n", currCycle);
    
    // See WB as the completion statge
	for (auto it = complSet.begin(); it != complSet.end(); it++) {
        wb_op = *it;
        DEBUG_MSG("Instruction Retires: Dst R%d (Phy R%d)\n", wb_op->reg_dst, wb_op->reg_phy_dst);
        pipeStageWb();
    }
    complSet.clear();
    /* Update ROB head pointer */
    reorderedBuffer.retire_insts();


	if(RUN_BIT == false)
    		return;
	//pipeStageMem();

    /* Keep tracking of execution list,
     * once the execution is done, complete that */
    std::list<Pipe_Op*> removeList;
    for (auto it = execSet.begin(); it != execSet.end(); it++) {
        auto op_ptr = *it;
        assert(op_ptr->exec_done);
        if (op_ptr->is_mem) {
            mem_op = op_ptr;
            pipeStageMem();
        }
        else {
            if (op_ptr->stall == 0) {
                OP_COMPL(op_ptr);
            }
            else {
                execute_op = op_ptr;
                pipeStageExecute();
            }
        }
        if (op_ptr->compl_done) {
            removeList.push_back(*it);
        }
    }
    /* Remove the instructions which are executed done */
    for (auto it = removeList.begin(); it != removeList.end(); it++) {
        execSet.erase(execSet.find(*it));
        complSet.insert(*it);
    }
    removeList.clear();

    /* Here doing the issue of instructiosns 
     * and do memory acess if the instruction is LOAD and store */
	for (int i = 0; i < RS_OP_NUM; i++) {
        if (!reservStation.rs_entries[i].busy) {
            continue;
        }
        if (!reservStation.rs_entries[i].src_reg1_ready || !reservStation.rs_entries[i].src_reg2_ready) {
            bool isReady = (mapTable.ready[reservStation.rs_entries[i].src_reg1] 
                && mapTable.ready[reservStation.rs_entries[i].src_reg2]);
            auto reg_phy_src1 = reservStation.rs_entries[i].src_reg1;
            auto reg_phy_src2 = reservStation.rs_entries[i].src_reg2;
            if (mapTable.ready[reg_phy_src1]) {
                reservStation.rs_entries[i].src_reg1_ready = true;
                ((Pipe_Op*)(reservStation.rs_entries[i].op_ptr))->reg_src1_value = mapTable.regValue[reg_phy_src1];
            }
            if (mapTable.ready[reg_phy_src2]) {
                reservStation.rs_entries[i].src_reg2_ready = true;
                ((Pipe_Op*)(reservStation.rs_entries[i].op_ptr))->reg_src2_value = mapTable.regValue[reg_phy_src2];
            }
            if (!isReady) {
                continue;
            }
        }
        execute_op = (Pipe_Op*)reservStation.rs_entries[i].op_ptr;
        
        /* should not be executed before */
        assert(!execute_op->exec_done);
        pipeStageExecute();
        
        if (i == LOAD || i == STORE) {
            mem_op = (Pipe_Op*)reservStation.rs_entries[i].op_ptr;
            pipeStageMem();
        }
        /* Free the entry */
        reservStation.rs_entries[i].busy = false;
        /* Insert to the set to track */
        execSet.insert((Pipe_Op*)reservStation.rs_entries[i].op_ptr);
	}
    pipeStageDecode();
	pipeStageFetch();

	//handle branch recoveries
	if (branch_recover) {
		DPRINTF(DEBUG_PIPE, "branch recovery: new dest %08x flush %d stages\n",
				branch_dest, branch_flush);

		PC = branch_dest;

		if (branch_flush >= 1) {
			if (fetch_op)
				free(fetch_op);
			fetch_op = nullptr;
		}

		if (branch_flush >= 2) {
			if (decode_op)
				free(decode_op);
			decode_op = nullptr;
		}

		if (branch_flush >= 3) {
			if (execute_op)
				free(execute_op);
			execute_op = nullptr;
		}

		if (branch_flush >= 4) {
			if (mem_op)
				free(mem_op);

			mem_op = nullptr;
		}

		if (branch_flush >= 5) {
			if (wb_op)
				free(wb_op);
			wb_op = nullptr;
		}

		branch_recover = 0;
		branch_dest = 0;
		branch_flush = 0;

		stat_squash++;
	}
}

void PipeState::pipeRecover(int flush, uint32_t dest) {
	/* if there is already a recovery scheduled, it must have come from a later
	 * stage (which executes older instructions), hence that recovery overrides
	 * our recovery. Simply return in this case. */
	if (branch_recover)
		return;
	//schedule the recovery. This will be done once all pipeline stages simulate the current cycle.
	branch_recover = 1;
	branch_flush = flush;
	branch_dest = dest;
}

void PipeState::pipeStageWb() {
	//if there is no instruction in this pipeline stage, we are done
	if (!wb_op)
		return;
	//grab the op out of our input slot
	Pipe_Op *op = wb_op;
	wb_op = NULL;

	//if this instruction writes a register, do so now
	if (op->reg_phy_dst != -1 && op->reg_phy_dst != 0) {
		//REGS[op->reg_dst] = op->reg_dst_value;
        
        // FIX_CHIA-HAO: 
        /* Update ArchTable */
        archMap.regMap[op->reg_dst] = op->reg_phy_dst;
        archMap.regValue[op->reg_phy_dst] = mapTable.regValue[op->reg_phy_dst];
        /* Update FreeList */
        freeList.isFree[op->reg_phy_dst_overwritten] = true;
        /* Delete ROB entry */
        auto entry_index = reorderedBuffer.entry_index_map[(void*)(op)];
        reorderedBuffer.ROB_entries[entry_index].ready_to_retire = true;
        
        DEBUG_MSG("Retire insts: update ArchMap %d : %d, Free %d\n", op->reg_dst, op->reg_phy_dst, op->reg_phy_dst_overwritten);
		DPRINTF(DEBUG_PIPE, "R%d = %08x\n", op->reg_dst, op->reg_dst_value);
	}
	//if this was a syscall, perform action
	if (op->opcode == OP_SPECIAL && op->subop == SUBOP_SYSCALL) {
		if (op->reg_src1_value == 0xA) {
			PC = op->pc; /* fetch will do pc += 4, then we stop with correct PC */
			RUN_BIT = false;
		}
	}

	//free the op
	free(op);
	stat_inst_retire++;
}

void PipeState::pipeStageMem() {
	//grab the op out of our input slot
	Pipe_Op *op = mem_op;

	//if there is no instruction in this pipeline stage, we are done
	if (!op)
		return;
	else {
		// FIX_CHIA-HAO: I dont expect there are operations other than load/store coming here
        assert(op->is_mem);
        /*
        if (op->is_mem == false) {
//			DPRINTF(DEBUG_PIPE, "clearing memory stage for instruction %x\n",
//					mem_op->pc);
			mem_op = NULL;
			wb_op = op;
			return;
		}
        */
		if (op->memTried == true) {
			if (op->waitOnPktIssue) {
				op->waitOnPktIssue = !(data_mem->sendReq(op->memPkt));
				return;
			}
			if (op->readyForNextStage == false)
				return;
			else {
//				DPRINTF(DEBUG_PIPE,
//						"clearing memory stage for instruction %x\n",
//						mem_op->pc);
				Pipe_Op* op = mem_op;
				mem_op = NULL;
				//wb_op = op;
                
                /* FIX_CHIA-HAO: Completion */
                OP_COMPL(op);
                
				return;
			}
		}
	}
	op->readyForNextStage = false;
	op->memTried = true;
	switch (op->opcode) {
	case OP_LW:
	case OP_LH:
	case OP_LHU:
	case OP_LB:
	case OP_LBU: {
		uint8_t* data = new uint8_t[4];
		op->memPkt = new Packet(true, false, PacketTypeLoad,
				(op->mem_addr & ~3), 4, data, currCycle);
		break;
	}
	case OP_SB: {
		uint8_t* data = new uint8_t;
		*data = op->mem_value & 0xFF;
		op->memPkt = new Packet(true, true, PacketTypeStore, (op->mem_addr), 1,
				data, currCycle);
		break;
	}
	case OP_SH: {
		uint16_t* data = new uint16_t;
		*data = op->mem_value & 0xFFFF;
		op->memPkt = new Packet(true, true, PacketTypeStore, (op->mem_addr), 2,
				(uint8_t*) data, currCycle);
		break;
	}

	case OP_SW: {
		uint32_t* data = new uint32_t;
		*data = op->mem_value;
		op->memPkt = new Packet(true, true, PacketTypeStore, (op->mem_addr), 4,
				(uint8_t*) data, currCycle);
		break;
	}
	}
	DPRINTF(DEBUG_PIPE,
			"sending pkt from memory stage: addr = %x, size = %d, type = %d \n",
			op->memPkt->addr, op->memPkt->size, op->memPkt->type);
	op->waitOnPktIssue = !(data_mem->sendReq(op->memPkt));
	return;
}

void PipeState::pipeStageExecute() {
	//if a multiply/divide is in progress, decrement cycles until value is ready
	if (execute_op && execute_op->stall > 0)
		execute_op->stall--;

	//if downstream stall, return (and leave any input we had)
	//if (mem_op != NULL)
	//	return;

	//if no op to execute, return
	if (execute_op == NULL)
		return;

	//grab op and read sources
	Pipe_Op *op = execute_op;

	//read register values, stall if necessary
	int stall = 0;
	if (op->reg_phy_src1 != -1) {
		if (op->reg_phy_src1 == 0)
			op->reg_src1_value = 0;
        /*
        else {
            op->reg_src1_value = MapTable.regValue[op->reg_phy_src1];
        }
        */
        /* FIX_CHIA-HAO: skipping forwarding here
		else if (mem_op && mem_op->reg_dst == op->reg_src1) {
			if (!mem_op->reg_dst_value_ready)
                stall = 1;
            else
                op->reg_src1_value = mem_op->reg_dst_value;
		} else if (wb_op && wb_op->reg_dst == op->reg_src1) {
			op->reg_src1_value = wb_op->reg_dst_value;
		} else
			op->reg_src1_value = REGS[op->reg_src1];
        */
	}
	if (op->reg_phy_src2 != -1) {
		if (op->reg_phy_src2 == 0)
			op->reg_src2_value = 0;
        /*
        else {
            op->reg_src2_value = regMap.regValue[op->reg_phy_src2];
        }
        */
        /*
		else if (mem_op && mem_op->reg_dst == op->reg_src2) {
			if (!mem_op->reg_dst_value_ready)
                stall = 1;
            else
                op->reg_src2_value = mem_op->reg_dst_value;

		} else if (wb_op && wb_op->reg_dst == op->reg_src2) {
			 op->reg_src2_value = wb_op->reg_dst_value;
		} else
			op->reg_src2_value = REGS[op->reg_src2];
	    */
    }

	//if requires a stall return without clearing stage input
	if (stall)
		return;
	//execute the op
	switch (op->opcode) {
	case OP_SPECIAL:
		op->reg_dst_value_ready = 1;
		switch (op->subop) {
		case SUBOP_SLL:
			op->reg_dst_value = op->reg_src2_value << op->shamt;
			break;
		case SUBOP_SLLV:
			op->reg_dst_value = op->reg_src2_value << op->reg_src1_value;
			break;
		case SUBOP_SRL:
			op->reg_dst_value = op->reg_src2_value >> op->shamt;
			break;
		case SUBOP_SRLV:
			op->reg_dst_value = op->reg_src2_value >> op->reg_src1_value;
			break;
		case SUBOP_SRA:
			op->reg_dst_value = (int32_t) op->reg_src2_value >> op->shamt;
			break;
		case SUBOP_SRAV:
			op->reg_dst_value = (int32_t) op->reg_src2_value
					>> op->reg_src1_value;
			break;
		case SUBOP_JR:
		case SUBOP_JALR:
			op->reg_dst_value = op->pc + 4;
			op->branch_dest = op->reg_src1_value;
			op->branch_taken = 1;
			break;

		case SUBOP_MULT: {
			/* we set a result value right away; however, we will
			 * model a stall if the program tries to read the value
			 * before it's ready (or overwrite HI/LO). Also, if
			 * another multiply comes down the pipe later, it will
			 * update the values and re-set the stall cycle count
			 * for a new operation.
			 */
			int64_t val = (int64_t) ((int32_t) op->reg_src1_value)
					* (int64_t) ((int32_t) op->reg_src2_value);
			uint64_t uval = (uint64_t) val;
			HI = (uval >> 32) & 0xFFFFFFFF;
			LO = (uval >> 0) & 0xFFFFFFFF;

			//four-cycle multiplier latency
			op->stall = 4;
		}
			break;
		case SUBOP_MULTU: {
			uint64_t val = (uint64_t) op->reg_src1_value
					* (uint64_t) op->reg_src2_value;
			HI = (val >> 32) & 0xFFFFFFFF;
			LO = (val >> 0) & 0xFFFFFFFF;

			//four-cycle multiplier latency
			op->stall = 4;
		}
			break;

		case SUBOP_DIV:
			if (op->reg_src2_value != 0) {

				int32_t val1 = (int32_t) op->reg_src1_value;
				int32_t val2 = (int32_t) op->reg_src2_value;
				int32_t div, mod;

				div = val1 / val2;
				mod = val1 % val2;

				LO = div;
				HI = mod;
			} else {
				//really this would be a div-by-0 exception
				HI = LO = 0;
			}

			//2-cycle divider latency
			op->stall = 32;
			break;

		case SUBOP_DIVU:
			if (op->reg_src2_value != 0) {
				HI = (uint32_t) op->reg_src1_value
						% (uint32_t) op->reg_src2_value;
				LO = (uint32_t) op->reg_src1_value
						/ (uint32_t) op->reg_src2_value;
			} else {
				/* really this would be a div-by-0 exception */
				HI = LO = 0;
			}

			/* 32-cycle divider latency */
			op->stall = 32;
			break;

		case SUBOP_MFHI:
			/* stall until value is ready */
			if (op->stall > 0)
				return;

			op->reg_dst_value = HI;
			break;
		case SUBOP_MTHI:
			//stall to respect WAW dependence
			if (op->stall > 0)
				return;

			HI = op->reg_src1_value;
			break;

		case SUBOP_MFLO:
			//stall until value is ready
			if (op->stall > 0)
				return;

			op->reg_dst_value = LO;
			break;
		case SUBOP_MTLO:
			//stall to respect WAW dependence
			if (op->stall > 0)
				return;

			LO = op->reg_src1_value;
			break;

		case SUBOP_ADD:
		case SUBOP_ADDU:
			op->reg_dst_value = op->reg_src1_value + op->reg_src2_value;
			break;
		case SUBOP_SUB:
		case SUBOP_SUBU:
			op->reg_dst_value = op->reg_src1_value - op->reg_src2_value;
			break;
		case SUBOP_AND:
			op->reg_dst_value = op->reg_src1_value & op->reg_src2_value;
			break;
		case SUBOP_OR:
			op->reg_dst_value = op->reg_src1_value | op->reg_src2_value;
			break;
		case SUBOP_NOR:
			op->reg_dst_value = ~(op->reg_src1_value | op->reg_src2_value);
			break;
		case SUBOP_XOR:
			op->reg_dst_value = op->reg_src1_value ^ op->reg_src2_value;
			break;
		case SUBOP_SLT:
			op->reg_dst_value =
					((int32_t) op->reg_src1_value < (int32_t) op->reg_src2_value) ?
							1 : 0;
			break;
		case SUBOP_SLTU:
			op->reg_dst_value =
					(op->reg_src1_value < op->reg_src2_value) ? 1 : 0;
			break;
		}
		break;

	case OP_BRSPEC:
		switch (op->subop) {
		case BROP_BLTZ:
		case BROP_BLTZAL:
			if ((int32_t) op->reg_src1_value < 0)
				op->branch_taken = 1;
			break;

		case BROP_BGEZ:
		case BROP_BGEZAL:
			if ((int32_t) op->reg_src1_value >= 0)
				op->branch_taken = 1;
			break;
		}
		break;

	case OP_BEQ:
		if (op->reg_src1_value == op->reg_src2_value)
			op->branch_taken = 1;
		break;

	case OP_BNE:
		if (op->reg_src1_value != op->reg_src2_value)
			op->branch_taken = 1;
		break;

	case OP_BLEZ:
		if ((int32_t) op->reg_src1_value <= 0)
			op->branch_taken = 1;
		break;

	case OP_BGTZ:
		if ((int32_t) op->reg_src1_value > 0)
			op->branch_taken = 1;
		break;

	case OP_ADDI:
	case OP_ADDIU:
		op->reg_dst_value_ready = 1;
		op->reg_dst_value = op->reg_src1_value + op->se_imm16;
		break;
	case OP_SLTI:
		op->reg_dst_value_ready = 1;
		op->reg_dst_value =
				(int32_t) op->reg_src1_value < (int32_t) op->se_imm16 ? 1 : 0;
		break;
	case OP_SLTIU:
		op->reg_dst_value_ready = 1;
		op->reg_dst_value =
				(uint32_t) op->reg_src1_value < (uint32_t) op->se_imm16 ? 1 : 0;
		break;
	case OP_ANDI:
		op->reg_dst_value_ready = 1;
		op->reg_dst_value = op->reg_src1_value & op->imm16;
		break;
	case OP_ORI:
		op->reg_dst_value_ready = 1;
		op->reg_dst_value = op->reg_src1_value | op->imm16;
		break;
	case OP_XORI:
		op->reg_dst_value_ready = 1;
		op->reg_dst_value = op->reg_src1_value ^ op->imm16;
		break;
	case OP_LUI:
		op->reg_dst_value_ready = 1;
		op->reg_dst_value = op->imm16 << 16;
		break;

	case OP_LW:
	case OP_LH:
	case OP_LHU:
	case OP_LB:
	case OP_LBU:
		op->mem_addr = op->reg_src1_value + op->se_imm16;
		break;

	case OP_SW:
	case OP_SH:
	case OP_SB:
		op->mem_addr = op->reg_src1_value + op->se_imm16;
		op->mem_value = op->reg_src2_value;
		break;
	}

	//update the branch predictor metadata
	BP->update(op->pc, op->branch_taken, op->branch_dest);
	//handle branch recoveries at this point
	if (op->branch_taken)
		pipeRecover(3, op->branch_dest);

    op->exec_done = true;
    DEBUG_MSG("Execute Instruction with opCode %d, reg_dst R%d\n", op->opcode, op->reg_dst);

	//remove from upstream stage and place in downstream stage
	execute_op = NULL;
	//mem_op = op;
}

void PipeState::pipeStageDecode() {
	//if downstream stall, return (and leave any input we had)
	//if (execute_op != NULL)
	//	return;

	//if no op to decode, return
	// FIX_CHIA-HAO: now you have 4 instructions in decode_op
    if (decode_op == NULL)
		return;

	//grab op and remove from stage input
	Pipe_Op *op = decode_op;
	// FIX_CHIA-HAO: when all of 4 instructions are decoded, decode_op will be set as NULL
    //decode_op = NULL;
    for (int i = 0; i < FETCH_INST_NUM; i++) {
        if (!op->inst_decoded_done[i]) {
            //set up info fields (source/dest regs, immediate, jump dest) as necessary
            uint32_t opcode = (op->instruction[i] >> 26) & 0x3F;
            uint32_t rs = (op->instruction[i] >> 21) & 0x1F;    
            uint32_t rt = (op->instruction[i] >> 16) & 0x1F;
            uint32_t rd = (op->instruction[i] >> 11) & 0x1F;
            uint32_t shamt = (op->instruction[i] >> 6) & 0x1F;
            uint32_t funct1 = (op->instruction[i] >> 0) & 0x1F;
            uint32_t funct2 = (op->instruction[i] >> 0) & 0x3F;
            uint32_t imm16 = (op->instruction[i] >> 0) & 0xFFFF;
            uint32_t se_imm16 = imm16 | ((imm16 & 0x8000) ? 0xFFFF8000 : 0);
            uint32_t targ = (op->instruction[i] & ((1UL << 26) - 1)) << 2;

            op->opcode = opcode;
            op->imm16 = imm16;
            op->se_imm16 = se_imm16;
            op->shamt = shamt;
            
            // FIX_CHIA-HAO: initialize to -1 first
            op->reg_src1 = -1;
            op->reg_src2 = -1;
            op->reg_dst = -1;
            op->reg_phy_src1 = -1;
            op->reg_phy_src2 = -1;
            op->reg_phy_dst = -1;

            RS_OP rs_op = INVALID;

            switch (opcode) {
            case OP_SPECIAL:
                /* all "SPECIAL" insts are R-types that use the ALU and both source
                 * regs. Set up source regs and immediate value. */
                op->reg_src1 = rs;
                op->reg_src2 = rt;
                op->reg_dst = rd;
                op->subop = funct2;
                if (funct2 == SUBOP_SYSCALL) {
                    op->reg_src1 = 2; // v0
                    op->reg_src2 = 3; // v1
                }
                if (funct2 == SUBOP_JR || funct2 == SUBOP_JALR) {
                    op->is_branch = 1;
                    op->branch_cond = 0;
                }
                
                rs_op = INT_ALU1;

                break;

            case OP_BRSPEC:
                //branches that have -and-link variants come here
                op->is_branch = 1;
                op->reg_src1 = rs;
                op->reg_src2 = rt;
                op->is_branch = 1;
                op->branch_cond = 1; /* conditional branch */
                op->branch_dest = op->pc + 4 + (se_imm16 << 2);
                op->subop = rt;
                if (rt == BROP_BLTZAL || rt == BROP_BGEZAL) {
                    /* link reg */
                    op->reg_dst = 31;
                    op->reg_dst_value = op->pc + 4;
                    op->reg_dst_value_ready = 1;
                }

                rs_op = INT_ALU1;

                break;

            case OP_JAL:
                op->reg_dst = 31;
                op->reg_dst_value = op->pc + 4;
                op->reg_dst_value_ready = 1;
                op->branch_taken = 1;
                //fallthrough
            case OP_J:
                op->is_branch = 1;
                op->branch_cond = 0;
                op->branch_taken = 1;
                op->branch_dest = (op->pc & 0xF0000000) | targ;
                
                rs_op = INT_ALU1;
                
                break;

            case OP_BEQ:
            case OP_BNE:
            case OP_BLEZ:
            case OP_BGTZ:
                //ordinary conditional branches (resolved after execute)
                op->is_branch = 1;
                op->branch_cond = 1;
                op->branch_dest = op->pc + 4 + (se_imm16 << 2);
                op->reg_src1 = rs;
                op->reg_src2 = rt;
                
                rs_op = INT_ALU1;

                break;

            case OP_ADDI:
            case OP_ADDIU:
            case OP_SLTI:
            case OP_SLTIU:
                //I-type ALU ops with sign-extended immediates
                op->reg_src1 = rs;
                op->reg_dst = rt;
                
                rs_op = INT_ALU1;
                
                break;

            case OP_ANDI:
            case OP_ORI:
            case OP_XORI:
            case OP_LUI:
                //I-type ALU ops with non-sign-extended immediates
                op->reg_src1 = rs;
                op->reg_dst = rt;
                
                rs_op = INT_ALU1;
                
                break;

            case OP_LW:
            case OP_LH:
            case OP_LHU:
            case OP_LB:
            case OP_LBU:
            case OP_SW:
            case OP_SH:
            case OP_SB:
                //memory ops
                op->is_mem = 1;
                op->reg_src1 = rs;
                if (opcode == OP_LW || opcode == OP_LH || opcode == OP_LHU
                        || opcode == OP_LB || opcode == OP_LBU) {
                    //load
                    op->mem_write = 0;
                    op->reg_dst = rt;
                    rs_op = LOAD;
                } else {
                    //store
                    op->mem_write = 1;
                    op->reg_src2 = rt;
                    rs_op = STORE;
                }
                break;
            }
                
            /* get src physical register */
            if (op->reg_src1 != -1) {
                if (op->reg_src1 == 0) {
                    op->reg_phy_src1 = 0;
                    op->reg_phy_src1_ready = true;
                }
                else {
                    op->reg_phy_src1 = mapTable.regMap[op->reg_src1];
                    op->reg_phy_src1_ready = mapTable.ready[op->reg_phy_src1];
                }
            }
            else {
                op->reg_phy_src1_ready = true;
            }

            if (op->reg_src2 != -1) {
                if (op->reg_src2 == 0) {
                    op->reg_phy_src2 = 0;
                    op->reg_phy_src2_ready = true;
                }
                else {
                    op->reg_phy_src2 = mapTable.regMap[op->reg_src2];
                    op->reg_phy_src2_ready = mapTable.ready[op->reg_phy_src2];
                }
            }
            else {
                op->reg_phy_src2_ready = true;
            }

            /* Request free physical registers for dst register */
            int reg_phy_dst = freeList.getNextFreeReg();
            if (reg_phy_dst == -1) 
                continue;
            else {
               op->reg_phy_dst_overwritten = archMap.regMap[op->reg_dst];
               op->reg_phy_dst = reg_phy_dst;
               mapTable.regMap[op->reg_dst] = reg_phy_dst;
               mapTable.ready[op->reg_phy_dst] = false;
            }

            /* Allocate RS */
            if (rs_op == INT_ALU1) {
                if (reservStation.rs_entries[INT_ALU1].busy) {
                    if (reservStation.rs_entries[INT_ALU2].busy) {
                        continue; 
                    }
                    else {
                        rs_op = INT_ALU2;
                    }
                }
            }
            else {
                if (reservStation.rs_entries[rs_op].busy) {
                    continue;
                }
            }
            
            /* Allocate ROB */
            int ROB_avail_index = reorderedBuffer.getNextAvail();
            if (ROB_avail_index == -1) continue;
                
            /* Allocate LSQ if needed */
            int LSQ_avail_index;    
            if (rs_op == LOAD || rs_op == STORE) {
                if ((LSQ_avail_index=ldstQueue.getLSQAvail()) == -1) { // full
                    continue;
                }
            } 

            /* Update everything */
            Pipe_Op *op_ptr = (Pipe_Op*)malloc(sizeof(Pipe_Op));
            memcpy(op_ptr, op, sizeof(Pipe_Op));
            op_ptr->exec_done = false;
            op_ptr->compl_done = false;
            /* Update LSQ, push to LSQ */
            if (rs_op == LOAD || rs_op == STORE) {
                ldstQueue.size++;
                ldstQueue.tail = LSQ_avail_index;
                ldstQueue.lsq_entries[LSQ_avail_index] = (void*)op_ptr;
            }
            /* Update Free list */ 
            freeList.isFree[op->reg_phy_dst] = false;
            /* Update Map Table */
            mapTable.regMap[op->reg_dst] = op->reg_phy_dst;
            /* Update Reservation Stations */
            reservStation.rs_entries[rs_op].busy = true;
            reservStation.rs_entries[rs_op].op_ptr = (void*)op_ptr;
            reservStation.rs_entries[rs_op].output_reg = op->reg_phy_dst;
            reservStation.rs_entries[rs_op].overwritten_reg = op->reg_phy_dst_overwritten;
            reservStation.rs_entries[rs_op].src_reg1 = op->reg_phy_src1;
            reservStation.rs_entries[rs_op].src_reg2 = op->reg_phy_src2;
            reservStation.rs_entries[rs_op].src_reg1_ready = op->reg_phy_src1_ready;
            reservStation.rs_entries[rs_op].src_reg2_ready = op->reg_phy_src2_ready;
            /* Update Reorder Buffer */
            reorderedBuffer.num_entries++;
            reorderedBuffer.tail = ROB_avail_index;
            reorderedBuffer.occupied[ROB_avail_index] = true;
            reorderedBuffer.ROB_entries[ROB_avail_index].op_ptr = (void*)op_ptr;
            reorderedBuffer.ROB_entries[ROB_avail_index].ready_to_retire = false;
            reorderedBuffer.ROB_entries[ROB_avail_index].output_reg = op->reg_phy_dst;
            reorderedBuffer.ROB_entries[ROB_avail_index].overwritten_reg = op->reg_phy_dst_overwritten;
            reorderedBuffer.entry_index_map[op_ptr] = ROB_avail_index;

            /* set this instruction is decoded done */
            op->inst_decoded_done[i] = true;
            DEBUG("Instruction Decoded: \n", op_ptr, i);
        }
    }
    
    if (OP_DEC_DONE(decode_op)) {
        delete decode_op;
        decode_op = NULL;
    }
    
    // place op in downstream slot
	//execute_op = op;
}

void PipeState::pipeStageFetch() {
	//if pipeline is stalled (our output slot is not empty), return
	
    if (decode_op != NULL)
		return;

	if (fetch_op != NULL) {
		if (fetch_op->isFetchIssued == false) {
			//if sending the packet was unsuccessful before, try again
			fetch_op->isFetchIssued = inst_mem->sendReq(fetch_op->instFetchPkt);
			return;
		}
		if (fetch_op->readyForNextStage == false)
			return;
		else {
			decode_op = fetch_op;
			fetch_op = nullptr;
			stat_inst_fetch++;
		}
	}

	assert(fetch_op == nullptr);
	fetch_op = (Pipe_Op *) malloc(sizeof(Pipe_Op));
	memset(fetch_op, 0, sizeof(Pipe_Op));

	fetch_op->reg_src1 = fetch_op->reg_src2 = fetch_op->reg_dst = -1;
	fetch_op->pc = PC;
	uint8_t* data = new uint8_t[4*FETCH_INST_NUM];
	fetch_op->instFetchPkt = new Packet(true, false, PacketTypeFetch, PC, 4*FETCH_INST_NUM,
			data, currCycle);
	DPRINTF(DEBUG_PIPE, "sending pkt from fetch stage with addr %x \n: ",
			fetch_op->instFetchPkt->addr);
	//try to send the memory request
	fetch_op->isFetchIssued = inst_mem->sendReq(fetch_op->instFetchPkt);
	//get the next instruction to fetch from branch predictor
	uint32_t target = BP->getTarget(PC);
	if (target == -1) {
		PC = PC + 4*FETCH_INST_NUM;
	} else {
		PC = target;
	}
}

bool PipeState::sendReq(Packet* pkt) {
	assert(false && "Nobody send request to core, Core is the boss :D");
	return true;
}

void PipeState::recvResp(Packet* pkt) {
	DPRINTF(DEBUG_PIPE,
			"core received a response for pkt : addr = %x, type = %d\n",
			pkt->addr, pkt->type);
	switch (pkt->type) {
	case PacketTypeFetch:
		//if the pkt-type is fetch proceed with fetching the instruction
		if (fetch_op != nullptr && fetch_op->pc == pkt->addr && pkt->size == 4) {
			//FIX_CHIA-HAO: copy 4 fetched instructions
            for (int i = 0; i < FETCH_INST_NUM; i++) {
                fetch_op->instruction[i] = *((uint32_t*) pkt->data + i);
			    fetch_op->inst_decoded_done[i] = false;
            }
            fetch_op->readyForNextStage = true;
		}
		break;
	case PacketTypeLoad: {
		//if pkt-type is load proceed with loading the data
		if (((mem_op->mem_addr & ~3) == pkt->addr) && pkt->size == 4) {
			uint32_t val = *((uint32_t*) pkt->data);
			//extract needed value
			mem_op->reg_dst_value_ready = 1;
			if (mem_op->opcode == OP_LW) {
				mem_op->reg_dst_value = val;
			} else if (mem_op->opcode == OP_LH || mem_op->opcode == OP_LHU) {
				if (mem_op->mem_addr & 2)
					val = (val >> 16) & 0xFFFF;
				else
					val = val & 0xFFFF;

				// sign-extend
				if (mem_op->opcode == OP_LH)
					val |= (val & 0x8000) ? 0xFFFF8000 : 0;

				mem_op->reg_dst_value = val;
			} else if (mem_op->opcode == OP_LB || mem_op->opcode == OP_LBU) {
				switch (mem_op->mem_addr & 3) {
				case 0:
					val = val & 0xFF;
					break;
				case 1:
					val = (val >> 8) & 0xFF;
					break;
				case 2:
					val = (val >> 16) & 0xFF;
					break;
				case 3:
					val = (val >> 24) & 0xFF;
					break;
				}

				// sign-extend
				if (mem_op->opcode == OP_LB)
					val |= (val & 0x80) ? 0xFFFFFF80 : 0;

				mem_op->reg_dst_value = val;
                // FIX_CHIA-HAO: 
                mem_op->reg_phy_dst_value = val;
			}
			mem_op->readyForNextStage = true;
		}
		break;
	}
	case PacketTypeStore:
		if (mem_op->mem_addr == pkt->addr) {
			mem_op->readyForNextStage = true;
		} else {
			assert(false && "Invalid store response from memory or cache");
		}
		break;
	default:
		assert(false && "Invalid response from memory or cache");
	}
	delete pkt;
}
