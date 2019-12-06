#ifndef __OOO_DATA_STRUCT__
#define __OOO_DATA_STRUCT__

#include <unordered_map>
#include <list>
#include <set>
#include <string.h>

typedef enum {
    INT_ALU1 = 0,
    INT_ALU2,
    FP_ADD,
    FP_MULT,
    LOAD,
    STORE, 
    RS_OP_NUM,
    INVALID
} RS_OP;


class RS 
{
public:
    class RS_Entry 
    {
    public:
        RS_Entry(): busy(false), op_ptr(NULL), src_reg1_ready(false), src_reg2_ready(false)     
        { }
        bool busy;
        void* op_ptr;
        int32_t output_reg; // T
        int32_t overwritten_reg; // Told
        int32_t src_reg1;
        bool src_reg1_ready;
        int32_t src_reg2;
        bool src_reg2_ready;
    };
    int getEntryIdx();
    RS_Entry rs_entries[RS_OP_NUM];
};



class MapTable 
{
public:
    /* logical <-> physical */
    std::unordered_map<int,int> regMap;
    /* physical reg : ready bit */
    std::unordered_map<int,bool> ready;
    /* physical reg : value */
    std::unordered_map<int,bool> regValue;
};

class ArchMap
{
public:
    /* logical <-> physical */
    std::unordered_map<int,int> regMap;
    /* physical reg : value */
    std::unordered_map<int,bool> regValue;
};



class ROB 
{
public:
    class ROB_Entry 
    {
    public:
        ROB_Entry() : op_ptr(NULL), ready_to_retire(false)
        { }
        void* op_ptr;
        bool ready_to_retire;
        int32_t output_reg; // T
        int32_t overwritten_reg; // Told
    };
    
    ROB(int32_t _num) : num_entries(_num), head(1), tail(0) { 
        ROB_entries = new ROB_Entry[_num+1];
        occupied = new bool[_num+1];
        memset(occupied, false, sizeof(bool)*(_num+1));
    }
    ~ROB() {
        delete[] ROB_entries;
    }
        
    int32_t getNextAvail() {
        int32_t next = (tail==num_entries) ? 1 : tail+1;
        if (occupied[next]) {
            return -1;
        }
        return next;
    }

    void retire_insts() {
        while(ROB_entries[head].ready_to_retire == true) {
            ROB_entries[head].ready_to_retire = false;
            head++;
            if (head > num_entries) head = 1;
        }
    }

    ROB_Entry *ROB_entries;
    bool *occupied;
    int32_t num_entries;
    int32_t head;
    int32_t tail;
    
    /* Pipe_Op* <-> index of ROB_entries */
    std::unordered_map<void*, int> entry_index_map;
};

class LSQ 
{
public:
     
    LSQ(int32_t _num) : num_entries(_num), head(0), tail(0), size(0) {
        lsq_entries = (void**)malloc(sizeof(void*)*_num); 
        lsq_ready = new bool[_num];
    }
    
    int32_t getLSQAvail() {
        if (size == num_entries) { // full   
            return -1;
        }
        return (tail == num_entries-1) ? 0 : tail+1;
    }

    void** lsq_entries;
    bool* lsq_ready;
    int32_t num_entries;
    int32_t size;
    int32_t head;
    int32_t tail;
};

class FreeList 
{
public:
    FreeList(int32_t _num) : num_phy_reg(_num) { 
        isFree = new bool[_num];        
        for (int i = 0; i < _num; i++) {
            isFree[i] = true;
        }
        curr_free_idx = 1;
    }

    ~FreeList() {
        delete[] isFree;
    }
    int32_t getNextFreeReg() {
        int32_t snapshot_idx = curr_free_idx;
        while (curr_free_idx < num_phy_reg && !isFree[curr_free_idx]) {
            curr_free_idx++;
            if (curr_free_idx == num_phy_reg) {
                curr_free_idx = 1;
            }
            if (snapshot_idx == curr_free_idx) { // no free reg anymore
                return -1;
            }
        }
        return (int32_t)(curr_free_idx);
    }
    int32_t num_phy_reg;
    int32_t curr_free_idx;
    bool* isFree;
};

#endif
