#ifndef SIM_PROC_H
#define SIM_PROC_H

#include <string>
#include <iostream>
#include <cstdint>

using namespace std;

typedef struct proc_params {
    unsigned long int rob_size;
    unsigned long int iq_size;
    unsigned long int width;
} proc_params;

struct cycle_count {
    int starting_cycle;
    int cycle_count;
};

struct instruction {
    uint32_t pc;
    int operation_type;
    int dest_reg;
    int src1_reg;
    int src2_reg;
    int inst_no;       
    int num_of_cycles;
    
    
    bool src1_ready;    
    bool src2_ready;  
    int src1_tag;     
    int src2_tag;
    int dst_tag;

    cycle_count fetch;
    cycle_count decode;
    cycle_count rename;
    cycle_count reg_read;
    cycle_count dispatch;
    cycle_count IS;
    cycle_count EX;
    cycle_count writeback;
    cycle_count ROB_count;

    bool decode_start;
    bool rename_start;
    bool reg_read_start;
    bool dispatch_start;
    bool issue_start;
    bool execute_start;
    bool writeback_start;
    bool retire_start;

};

struct issue_queue {
    bool valid;          

    instruction inst;
};

struct rob_entry {
    int dst;     
    int inst_no;     
    bool ready;         
    uint32_t pc;       
    instruction inst;
};

struct renamed_reg {
    int value;     
    bool is_rob; 
};

struct rename_table {
    int reg_num;
    bool valid;
    int rob_tag;
};

class pipeline_reg {
private:
    proc_params params;
    instruction* decode;         
    instruction* rename;         
    instruction* reg_read;       
    instruction* dispatch;       
    issue_queue* issuequeue;            
    instruction* execute_list;   
    instruction* writeback;    
    instruction* retire;
    rob_entry* ROB;             
    rename_table* RMT;          
    
    int head;   
    int tail; 
    int count;

public:
    pipeline_reg(proc_params p, FILE* FP);
    ~pipeline_reg();

    FILE* trace_file;

    unsigned long current_cycle;
    int instr_count;   
    int decode_size;
    int rename_size;
    int reg_read_size;
    int dispatch_size;
    int issuequeue_size;
    int execute_list_size;
    int writeback_size;
    int retire_size;
    int ROB_size;

    bool fetch_inst();
    void decode_inst();
    void rename_inst();
    void reg_read_inst();
    void dispatch_inst();
    void issue_inst();
    void execute_inst();
    void writeback_inst();
    void retire_inst();
    bool adv_cycle();
    void run_pipeline(pipeline_reg& pipeline);
};

#endif
