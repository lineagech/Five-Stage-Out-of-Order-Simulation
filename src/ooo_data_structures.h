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
    void clear()
    {
        regMap.clear();
        ready.clear();
        regValue.clear();
    }
    
    /* logical <-> physical */
    std::unordered_map<int,int> regMap;
    /* physical reg : ready bit */
    std::unordered_map<int,bool> ready;
    /* physical reg : value */
    std::unordered_map<int,int> regValue;
};

class ArchMap
{
public:
    void clear()
    {
        regMap.clear();
        regValue.clear();
    }
    
    /* logical <-> physical */
    std::unordered_map<int,int> regMap;
    /* physical reg : value */
    std::unordered_map<int,int> regValue;
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
        if (occupied) {
            delete[] occupied;
            occupied = NULL;
        }
        if (ROB_entries) {
            delete[] ROB_entries;
            ROB_entries = NULL;
        }
    }
        
    int32_t getNextAvail() {
        int32_t next = (tail==num_entries) ? 1 : tail+1;
        if (occupied[next]) {
            return -1;
        }
        return next;
    }

    bool retire_insts() {
        while (ROB_entries[head].ready_to_retire == true) {
            ROB_entries[head].ready_to_retire = false;
            ROB_entries[head].op_ptr = NULL;
            occupied[head] = false;
            head++;
            if (head > num_entries) head = 1;
            //return true;
        }
        //return false;
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
        for (int i = 0; i < _num; i++) {
            lsq_entries[i] = NULL;
        }
        lsq_ready = new bool[_num];
    }
    
    int32_t getLSQAvail() {
        if (size == num_entries) { // full   
            return -1;
        }
        return (tail == num_entries-1) ? 0 : tail+1;
    }
    void retire() {
        lsq_entries[head] = NULL;
        lsq_ready[head] = true;
        size--;
        head = ((head+1) == num_entries) ? 0 : head+1;
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
    FreeList():num_phy_reg(0),curr_free_idx(0),isFree(NULL)
    { }
    
    FreeList(int32_t _num) : num_phy_reg(_num) { 
        isFree = new bool[_num];        
        for (int i = 0; i < _num; i++) {
            isFree[i] = true;
        }
        curr_free_idx = 33;
    }

    ~FreeList() {
        if (isFree) {
            delete[] isFree;
            isFree = NULL;
        }
    }
    
    void clear(){
       num_phy_reg = 0;
       curr_free_idx = 0;
       if (isFree) {
           delete[] isFree;
           isFree = NULL;
       }
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

class BranchStack
{
public:
    class BS_Entry
    {
    public:
        BS_Entry():ROB_tail(0),LSQ_tail(0)
        { }
        
        BS_Entry(const MapTable &maptable, const ArchMap &archmap, const FreeList &freelist, int32_t ROBtail, int32_t LSQtail)
        {
            //copy maptable
            br_maptable.regMap = maptable.regMap;
            br_maptable.ready = maptable.ready;
            br_maptable.regValue = maptable.regValue;
            
            //copy archmap
            br_archmap.regMap = archmap.regMap;
            br_archmap.regValue = br_archmap.regValue;
            
            //copy freelist
            br_freelist.num_phy_reg = freelist.num_phy_reg;
            br_freelist.curr_free_idx = freelist.curr_free_idx;
            br_freelist.isFree = new bool[br_freelist.num_phy_reg];
            for(int i = 0; i < br_freelist.num_phy_reg; i++)
            {
                br_freelist.isFree[i] = freelist.isFree[i];
            }
            
            //copy ROBtail and LSQ_tail
            ROB_tail = ROBtail;
            LSQ_tail = LSQtail;
        }
        
        MapTable br_maptable;
        ArchMap br_archmap;
        FreeList br_freelist;
        int32_t ROB_tail;
        int32_t LSQ_tail;
        
    };
    
    BranchStack(int32_t _num) : num_entries(_num),top_br(-1) {
        br_stack = new BS_Entry[num_entries];
        bmask = new bool[num_entries];
        memset(bmask, false, sizeof(bool)*(num_entries));
    }
    
    ~BranchStack() {
        if(br_stack)
        {
            delete[] br_stack;
            br_stack = NULL;
        }
        if(bmask)
        {
            delete[] bmask;
            bmask = NULL;
        }
    }
    
    void clearEntry(int32_t br_id)
    {
        for(int i = top_br; i >= br_id; i--)//remove nested checkpoint
        {
            br_stack[i].br_maptable.clear();
            br_stack[i].br_archmap.clear();
            br_stack[i].br_freelist.clear();
            br_stack[i].ROB_tail = 0;
            br_stack[i].LSQ_tail = 0;
            
            bmask[i] = false;
        }
    }
    
    BS_Entry* br_stack;
    bool* bmask;
    int32_t top_br;
    int32_t num_entries;
    
};

#endif
