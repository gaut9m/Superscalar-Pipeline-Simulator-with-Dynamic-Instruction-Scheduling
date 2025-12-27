#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim_proc.h"
#include <iomanip>

string trace_name;

pipeline_reg::pipeline_reg(proc_params p, FILE* FP) {
    params = p;
    trace_file = FP;
    current_cycle = 0;
    instr_count = 0;

    decode = new instruction[params.width];
    rename = new instruction[params.width];
    reg_read = new instruction[params.width];
    dispatch = new instruction[params.width];
    issuequeue = new issue_queue[params.iq_size];
    execute_list = new instruction[params.width * 5];
    writeback = new instruction[params.width * 5];
    ROB = new rob_entry[params.rob_size];
    RMT = new rename_table[67];

    decode_size = 0;
    rename_size = 0;
    reg_read_size = 0;
    dispatch_size = 0;
    issuequeue_size = 0;
    execute_list_size = 0;
    writeback_size = 0;
    retire_size = 0;
    ROB_size = 0;
    head = 0;
    tail = 0;
    count = 0;

    for(int i = 0; i < 67; i++) {
        RMT[i].reg_num = i;
        RMT[i].valid = false;
        RMT[i].rob_tag = -1;
    }
}

pipeline_reg::~pipeline_reg() {
    delete[] decode;
    delete[] rename;
    delete[] reg_read;
    delete[] dispatch;
    delete[] issuequeue;
    delete[] execute_list;
    delete[] writeback;
    delete[] ROB;
    delete[] RMT;
}





void pipeline_reg::run_pipeline(pipeline_reg& pipeline) {
    do {
        pipeline.retire_inst();
        pipeline.writeback_inst();
        pipeline.execute_inst();
        pipeline.issue_inst();
        pipeline.dispatch_inst();
        pipeline.reg_read_inst(); 
        pipeline.rename_inst();    
        pipeline.decode_inst();
        pipeline.fetch_inst();
        
    } while (pipeline.adv_cycle());
}


bool pipeline_reg::adv_cycle() {
    this->current_cycle++;

    if ((this->decode_size == 0 && 
         this->rename_size == 0 &&
         this->reg_read_size == 0 &&
         this->dispatch_size == 0 && 
         this->issuequeue_size == 0 &&
         this->execute_list_size == 0 && 
         this->writeback_size == 0 &&
         this->ROB_size == 0) && 
        (feof(this->trace_file))) { 

        float inst_per_cycle = static_cast<float>(instr_count)/static_cast<float>(current_cycle);

        cout << "# === Simulator Command =========" <<endl;
        cout << "# " << "./sim " << params.rob_size << " " << params.iq_size << " " << params.width << " " << trace_name <<endl;
        cout << "# === Processor Configuration ===" << endl;
        cout << "# ROB_SIZE = " << params.rob_size << endl;
        cout << "# IQ_SIZE  = " << params.iq_size << endl;
        cout << "# WIDTH    = " << params.width << endl;
        cout << "# === Simulation Results ========" << endl;
        cout <<"# Dynamic Instruction Count    = " << instr_count << endl;
        cout <<"# Cycles                       = " << current_cycle << endl;
        cout <<"# Instructions Per Cycle (IPC) = " << fixed << setprecision(2) << inst_per_cycle << endl;


        return false;  
    }
    
    return true;  
}


bool pipeline_reg::fetch_inst() {
    if (feof(trace_file) || decode_size >0) {
        return true;
    }

    while (decode_size < params.width) {
        instruction IN;
        int result = fscanf(trace_file, "%x %d %d %d %d", 
                          &IN.pc, 
                          &IN.operation_type, 
                          &IN.dest_reg,
                          &IN.src1_reg,
                          &IN.src2_reg);
        
        if (result != 5) {
            return (decode_size > 0);
        }

        if (result != 5 && decode_size > 0) {  
            return true;
        }

        IN.inst_no = instr_count;
        IN.decode_start = false;
        IN.rename_start = false;
        IN.reg_read_start = false;
        IN.dispatch_start = false;
        IN.issue_start = false;
        IN.execute_start = false;
        IN.writeback_start = false;
        IN.retire_start = false;
        IN.fetch.starting_cycle = current_cycle;
        instr_count++;
        decode[decode_size] = IN;
        decode_size++;
    } 
    return true;
}

void pipeline_reg::decode_inst() {
    for (int i=0; i<decode_size;++i) {
        if(!decode[i].decode_start) {
            decode[i].decode.starting_cycle = current_cycle;
            decode[i].decode_start=true;
            decode[i].fetch.cycle_count = decode[i].decode.starting_cycle - decode[i].fetch.starting_cycle;
        }
    }

    if (rename_size > 0 || decode_size == 0) {
        return;
    }


    int transfer_size;
    if(decode_size >= params.width) {
        transfer_size = params.width;
    } else {
        transfer_size = decode_size;
    }
    

    for(int i = 0; i < transfer_size; i++) {
        rename[rename_size] = decode[i];
        rename_size++;
        
    }

    for(int i = transfer_size; i < decode_size; i++) {
        decode[i - transfer_size] = decode[i];
    }
    decode_size = decode_size - transfer_size;
}

void pipeline_reg::rename_inst() {
    for (int i=0; i<rename_size;++i) {
        if(!rename[i].rename_start) {
            rename[i].rename.starting_cycle = current_cycle;
            rename[i].rename_start=true;
            rename[i].decode.cycle_count = rename[i].rename.starting_cycle - rename[i].decode.starting_cycle;
        }
    }
    int free_rob_entries = params.rob_size - ROB_size;

    if (rename_size == 0 || reg_read_size > 0 || free_rob_entries < rename_size) {
        return;
    }

    int transfer_size;
    if(rename_size == params.width){
        transfer_size = params.width;
    }
    if(rename_size < params.width){
        transfer_size = rename_size;
    }

    for (int i = 0; i < transfer_size; i++) {
        rename[i].src1_tag = -1;
        rename[i].src1_ready = true;
        if (rename[i].src1_reg != -1) {
            if (RMT[rename[i].src1_reg].valid) {
                rename[i].src1_tag = RMT[rename[i].src1_reg].rob_tag;
                rename[i].src1_ready = ROB[RMT[rename[i].src1_reg].rob_tag].ready;

            }
        }

        rename[i].src2_tag = -1;
        rename[i].src2_ready = true;
        if (rename[i].src2_reg != -1) {
            if (RMT[rename[i].src2_reg].valid) {
                rename[i].src2_tag = RMT[rename[i].src2_reg].rob_tag;
                rename[i].src2_ready = ROB[RMT[rename[i].src2_reg].rob_tag].ready;
            }
        }

        rename[i].dst_tag = tail;

        if (rename[i].dest_reg != -1) {
            RMT[rename[i].dest_reg].valid = true;
            RMT[rename[i].dest_reg].rob_tag = tail;
        }

        reg_read[reg_read_size] = rename[i];
        reg_read_size++;

        ROB[tail].ready = false;

        ROB[tail].inst = rename[i];
        tail = (tail + 1) % params.rob_size;
        ROB_size++;
    }


    for (int i = transfer_size; i < rename_size; i++) {
        rename[i - transfer_size] = rename[i];
    }
    rename_size = rename_size - transfer_size;
}





void pipeline_reg::reg_read_inst() {
    
    for (int i=0;i<reg_read_size;i++){
        if (!reg_read[i].reg_read_start){
        reg_read[i].reg_read.starting_cycle = current_cycle;
        reg_read[i].reg_read_start = true;
        reg_read[i].rename.cycle_count = reg_read[i].reg_read.starting_cycle - reg_read[i].rename.starting_cycle;
        }
    }

    if(reg_read_size == 0 || dispatch_size > 0) {  
        return;
    }


    int transfer_size;
    if(reg_read_size >= params.width){
        transfer_size = params.width;
    }
    else{
        transfer_size = reg_read_size;
    }

    for(int i = 0; i < transfer_size; i++) {
        if(reg_read[i].src1_reg != -1) {
            if(!reg_read[i].src1_ready && ROB[reg_read[i].src1_tag].ready) {
                reg_read[i].src1_ready = true;
            }

        }

        else {
            reg_read[i].src1_ready = true;
        }

        if(reg_read[i].src2_reg != -1) {
            if(!reg_read[i].src2_ready && ROB[reg_read[i].src2_tag].ready) {
                reg_read[i].src2_ready = true;
            }

        }
        else {
            reg_read[i].src2_ready = true;
        }

        dispatch[dispatch_size] = reg_read[i];
        dispatch_size++;
    }

    for(int i = transfer_size; i < reg_read_size; i++) {
        reg_read[i - transfer_size] = reg_read[i];
    }
    reg_read_size = reg_read_size - transfer_size;
}

void pipeline_reg::dispatch_inst() {
    for(int i=0;i<dispatch_size;++i){
        if(!dispatch[i].dispatch_start){
        dispatch[i].dispatch.starting_cycle = current_cycle;
        dispatch[i].dispatch_start = true;
        dispatch[i].reg_read.cycle_count = dispatch[i].dispatch.starting_cycle - dispatch[i].reg_read.starting_cycle;
        }
    }
    if(dispatch_size == 0) {
        return;
    }

    int free_iq_entries = params.iq_size - issuequeue_size;
    if(free_iq_entries < dispatch_size) {
        return;
    }


    for(int i = 0; i < dispatch_size; i++) {
        issuequeue[issuequeue_size].valid = true;
        issuequeue[issuequeue_size].inst = dispatch[i];
        issuequeue_size++;
    }

    dispatch_size = 0;
}

void pipeline_reg::issue_inst() {
    for(int i = 0; i < issuequeue_size; i++) {
        if(!issuequeue[i].inst.issue_start && issuequeue[i].inst.dispatch_start) {
            issuequeue[i].inst.IS.starting_cycle = current_cycle;
            issuequeue[i].inst.issue_start = true;
            issuequeue[i].inst.dispatch.cycle_count = issuequeue[i].inst.IS.starting_cycle - issuequeue[i].inst.dispatch.starting_cycle;
        }
    }

    if(issuequeue_size == 0 || execute_list_size >= params.width * 5) {
        return;
    }

    int issued_instructions = 0;
    while(issued_instructions < params.width && execute_list_size < params.width * 5) {
        int ready_instr = -1;
        for(int i = 0; i < issuequeue_size; i++) {
            if(issuequeue[i].valid && issuequeue[i].inst.src1_ready && issuequeue[i].inst.src2_ready) {
                ready_instr = i;
                break;
            }
        }
        
        if(ready_instr == -1) break;

        execute_list[execute_list_size] = issuequeue[ready_instr].inst;
        switch(issuequeue[ready_instr].inst.operation_type) {
            case 0: execute_list[execute_list_size].num_of_cycles = 1; break;
            case 1: execute_list[execute_list_size].num_of_cycles = 2; break;
            case 2: execute_list[execute_list_size].num_of_cycles = 5; break;
        }
        execute_list_size++;
        issued_instructions++;

        for(int j = ready_instr; j < issuequeue_size - 1; j++) {
            issuequeue[j] = issuequeue[j + 1];
        }
        issuequeue_size--;
    }
}



void pipeline_reg::execute_inst() {
    for(int i = 0; i<execute_list_size;i++){
        if(!execute_list[i].execute_start && execute_list[i].issue_start){
            execute_list[i].EX.starting_cycle = current_cycle;
            execute_list[i].execute_start = true;
            execute_list[i].IS.cycle_count = execute_list[i].EX.starting_cycle - execute_list[i].IS.starting_cycle;
        }
    }

    if (execute_list_size == 0 || writeback_size >= params.width * 5) {
        return;
    }

    for (int i = 0; i < execute_list_size; i++) {

        execute_list[i].num_of_cycles--;
        

        if (execute_list[i].num_of_cycles == 0) {
            if (writeback_size >= params.width * 5) {
                return;
            }
            
            for (int j = 0; j < issuequeue_size; j++) {
                if (issuequeue[j].inst.src1_tag == execute_list[i].dst_tag) {
                    issuequeue[j].inst.src1_ready = true;
                }
                if (issuequeue[j].inst.src2_tag == execute_list[i].dst_tag) {
                    issuequeue[j].inst.src2_ready = true;
                }
            }

            for (int j = 0; j < dispatch_size; j++) {
                if (dispatch[j].src1_tag == execute_list[i].dst_tag) {
                    dispatch[j].src1_ready = true;
                }
                if (dispatch[j].src2_tag == execute_list[i].dst_tag) {
                    dispatch[j].src2_ready = true;
                }
            }

            for (int j = 0; j < reg_read_size; j++) {
                if (reg_read[j].src1_tag == execute_list[i].dst_tag) {
                    reg_read[j].src1_ready = true;
                }
                if (reg_read[j].src2_tag == execute_list[i].dst_tag) {
                    reg_read[j].src2_ready = true;
                }
            }

            writeback[writeback_size] = execute_list[i];
            writeback_size++;



            for (int j = i; j < execute_list_size - 1; j++) {
                execute_list[j] = execute_list[j + 1];
            }
            execute_list_size--;
            i--; 
        }
    }


}



void pipeline_reg::writeback_inst() {
    for(int i =0; i < writeback_size; i++){
        if(!writeback[i].writeback_start && writeback[i].execute_start)
        {
        writeback[i].writeback.starting_cycle = current_cycle;
        writeback[i].writeback_start = true;
        writeback[i].EX.cycle_count = writeback[i].writeback.starting_cycle - writeback[i].EX.starting_cycle;
        }
        

    }


    for(int i = 0; i < writeback_size; i++) {
        int destination = writeback[i].dst_tag;
        ROB[destination].ready=true;
        ROB[destination].inst = writeback[i];
    }
    writeback_size = 0;
}

void pipeline_reg::retire_inst() {

    for(int i=0; i < params.rob_size; i++){
        if(ROB[i].ready){
            if (!ROB[i].inst.retire_start) {
                ROB[i].inst.ROB_count.starting_cycle = current_cycle;
                ROB[i].inst.retire_start = true;
                ROB[i].inst.writeback.cycle_count = 1;
            }

            for (int j = 0; j < dispatch_size; j++) {
                if (dispatch[j].src1_tag == i) {
                    dispatch[j].src1_ready = true;
                }
                if (dispatch[j].src2_tag == i) {
                    dispatch[j].src2_ready = true;
                }
            }

            for (int j = 0; j < reg_read_size; j++) {
                if (reg_read[j].src1_tag == i) {
                    reg_read[j].src1_ready = true;
                }
                if (reg_read[j].src2_tag == i) {
                    reg_read[j].src2_ready = true;
                }
            }

        }
        
    }

    if(!ROB[head].ready) {
        return;
    }

    int retired = 0;
    while(retired < params.width && ROB[head].ready) {

            ROB[head].ready = false;
            ROB[head].inst.ROB_count.cycle_count = current_cycle +1 - ROB[head].inst.ROB_count.starting_cycle;            
            cout << count << " fu{" << ROB[head].inst.operation_type << "} "
            << "src{" << ROB[head].inst.src1_reg << "," << ROB[head].inst.src2_reg << "} "
            << "dst{" << ROB[head].inst.dest_reg << "} "
            << "FE{" << ROB[head].inst.fetch.starting_cycle << "," << ROB[head].inst.fetch.cycle_count << "} "
            << "DE{" << ROB[head].inst.decode.starting_cycle << "," << ROB[head].inst.decode.cycle_count << "} "
            << "RN{" << ROB[head].inst.rename.starting_cycle << "," << ROB[head].inst.rename.cycle_count << "} "
            << "RR{" << ROB[head].inst.reg_read.starting_cycle << "," << ROB[head].inst.reg_read.cycle_count << "} "
            << "DI{" << ROB[head].inst.dispatch.starting_cycle << "," << ROB[head].inst.dispatch.cycle_count << "} "
            << "IS{" << ROB[head].inst.IS.starting_cycle << "," << ROB[head].inst.IS.cycle_count << "} "
            << "EX{" << ROB[head].inst.EX.starting_cycle << "," << ROB[head].inst.EX.cycle_count << "} "
            << "WB{" << ROB[head].inst.writeback.starting_cycle << "," << ROB[head].inst.writeback.cycle_count << "} "
            << "RT{" << ROB[head].inst.ROB_count.starting_cycle << "," << ROB[head].inst.ROB_count.cycle_count << "}" << endl;


            count++;

            if(ROB[head].inst.dest_reg != -1) {
                for(int i =0; i<67; i++){
                if(RMT[i].valid && RMT[i].rob_tag == head) {
                    RMT[i].valid = false;
                    RMT[i].rob_tag = -1;

                }
                }
        }

            head = (head + 1) % params.rob_size;  

        retired++;
        ROB_size--;          


        
    }
}


int main(int argc, char* argv[]) {
    FILE *FP;
    char *trace_file;
    proc_params params;

    
    if (argc != 5) {
        printf("Error: Wrong number of inputs:%d\n", argc-1);
        exit(EXIT_FAILURE);
    }
    
    params.rob_size = strtoul(argv[1], NULL, 10);
    params.iq_size = strtoul(argv[2], NULL, 10);
    params.width = strtoul(argv[3], NULL, 10);
    trace_file = argv[4];

    trace_name = trace_file;
    
    FP = fopen(trace_file, "r");
    if(FP == NULL) {
        printf("Error: Unable to open file %s\n", trace_file);
        exit(EXIT_FAILURE);
    }

    pipeline_reg pipeline(params, FP);
    int cycle_count =0;

    pipeline.run_pipeline(pipeline);



    fclose(FP);
    return 0;
}