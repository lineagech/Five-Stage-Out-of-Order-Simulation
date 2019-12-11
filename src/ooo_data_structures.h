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
        
        //Qian-bmask
        bool bmask[4];
    };
    int getEntryIdx();
    
    void clearEntries(int32_t br)
    {
        for (int i = 0; i < RS_OP_NUM; i++)
        {
            auto entry = &(rs_entries[i]);
            if(entry->bmask[br])
            {
                entry->busy = false;
                entry->op_ptr = NULL;
                entry->src_reg1_ready = false;
                entry->src_reg2_ready = false;
                for(int j = 0; j < 4; j++)
                    entry->bmask[j] = false;
            }
        }
    }
        
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
    
    MapTable &operator=(const MapTable &maptable)
    {
        if(this == &maptable) return *this;
        regMap = maptable.regMap;
        ready = maptable.ready;
        regValue = maptable.regValue;
        return *this;
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
    
    ArchMap &operator=(const ArchMap &archmap)
    {
        if(this == &archmap) return *this;
        regMap = archmap.regMap;
        regValue = archmap.regValue;
        return *this;
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
    
    void* getHead() {
        if (!occupied[head]) {
            return NULL;
        }
        return ROB_entries[head].op_ptr;
    }

    int32_t getNextAvail() {
        int32_t next = (tail==num_entries) ? 1 : tail+1;
        if (occupied[next]) {
            return -1;
        }
        return next;
    }

    bool retire_insts() {
        if (ROB_entries[head].ready_to_retire == true) {
            ROB_entries[head].ready_to_retire = false;
            ROB_entries[head].op_ptr = NULL;
            occupied[head] = false;
            head++;
            if (head > num_entries) head = 1;
            return true;
        }
        return false;
    }

    void backToOldTail(int old_tail) {
        while (tail != old_tail) {
            occupied[tail] = false;
            tail = (tail == 1) ? num_entries : tail-1;
        }
    }

    ROB_Entry *ROB_entries;
    bool *occupied;
    int32_t num_entries;
    int32_t head;
    int32_t tail;
    
    /* Pipe_Op* <-> index of ROB_entries */
    std::unordered_map<int64_t, int> entry_index_map;
};

class LSQ 
{
public:
     
    LSQ(int32_t _num) : num_entries(_num), head(0), tail(-1), size(0) {
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
    int getIndex(void* _ptr) {
        for (int i = 0; i < num_entries; i++) {
            if ((uint64_t)_ptr == (uint64_t)(lsq_entries[i])) {
                return i;           
            }
        }
        return -1;
    }
    void backToOldTail(int old_tail) {
       while (tail != old_tail) {
            lsq_entries[tail] = NULL;
            tail = (tail == 0) ? num_entries-1 : tail-1;
            size--;
        }
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
    FreeList():num_phy_reg(0),curr_free_idx(33),isFree(NULL)
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
       curr_free_idx = 33;
   }
    
    FreeList &operator=(const FreeList &freelist)
    {
        if(this == &freelist) return *this;
        num_phy_reg = freelist.num_phy_reg;
        curr_free_idx = freelist.curr_free_idx;
        
        if(isFree == NULL)
        {
            isFree = new bool[freelist.num_phy_reg];
        }
        
        for(int i = 0; i < freelist.num_phy_reg; i++)
        {
            isFree[i] = freelist.isFree[i];
        }
        return *this;
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

class BS_Entry
{
public:
    BS_Entry():ROB_tail(-1),LSQ_tail(-1)
    { }
    
    /*BS_Entry(const MapTable &maptable, const ArchMap &archmap, const FreeList &freelist, int32_t ROBtail, int32_t LSQtail):ROB_tail(ROBtail),LSQ_tail(LSQtail)
    {
        br_maptable = maptable;
        br_archmap = archmap;
        br_freelist = freelist;
    }
    
    BS_Entry operator=(const BS_Entry &bsEntry)
    {
        if(this == &bsEntry) return *this;
        br_maptable = bsEntry.br_maptable;
        br_archmap = bsEntry.br_archmap;
        br_freelist = bsEntry.br_freelist;
        ROB_tail = bsEntry.ROB_tail;
        LSQ_tail = bsEntry.LSQ_tail;
        return *this;
    }*/
    
    MapTable br_maptable;
    ArchMap br_archmap;
    FreeList br_freelist;
    int32_t ROB_tail;
    int32_t LSQ_tail;
    int32_t ROB_index; 
};


class BranchStack
{
public:
    
    /*BranchStack(int32_t _num) : num_entries(_num),head(0),tail(-1) {
        br_stack = new BS_Entry[num_entries];
        bmask = new bool[num_entries];
        memset(bmask, false, sizeof(bool)*(num_entries));
    }*/
    BranchStack():head(0),tail(-1)
    {
    }
    
    void clearEntries(int32_t br_id)
    {
        int i = tail;
        while(i != br_id)
        {
            if(bmask[i])
            {
                clearSingleEntry(i);
            }
            i--;
            if(i == -1) i = 3;
        }
        
        clearSingleEntry(i);
        tail = br_id-1;
    }
    
    void clearSingleEntry(int32_t i)
    {
        br_stack[i].br_maptable.clear();
        br_stack[i].br_archmap.clear();
        br_stack[i].br_freelist.clear();
        br_stack[i].ROB_tail = -1;
        br_stack[i].LSQ_tail = -1;
        
        bmask[i] = false;
    }
    
    void moveToNewHead(int32_t br)
    {
        while(br != tail && bmask[br])
        {
            br++;
            if(br == 4) br = 0;
        }
        head = br;
        
        if(head == tail && !bmask[head])
        {
            head = 0;
        }
    }
    
    int32_t getNextAvail() {
        int32_t next = (tail==3) ? 0 : tail+1;
        if (bmask[next]) {
            return -1;
        }
        return next;
    }
    
    BS_Entry br_stack[4];
    bool bmask[4];
    int32_t head;
    int32_t tail;
    //int32_t num_entries;
    
};

#endif
