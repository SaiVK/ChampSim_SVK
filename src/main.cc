#define _BSD_SOURCE

#include <getopt.h>
#include "ooo_cpu.h"
#include "uncore.h"


uint8_t warmup_complete[NUM_CPUS], 
        simulation_complete[NUM_CPUS], 
        all_warmup_complete = 0, 
        all_simulation_complete = 0,
        MAX_INSTR_DESTINATIONS = NUM_INSTR_DESTINATIONS,
        knob_cloudsuite = 0,
        knob_low_bandwidth = 0;

uint64_t warmup_instructions     = 1000000,
         simulation_instructions = 10000000,
         champsim_seed;

time_t start_time;

// PAGE TABLE
uint32_t PAGE_TABLE_LATENCY = 0, SWAP_LATENCY = 0;
uint64_t previous_ppage, num_adjacent_page, num_cl[NUM_CPUS], allocated_pages, num_page[NUM_CPUS], minor_fault[NUM_CPUS], major_fault[NUM_CPUS];

map<uint16_t,uint64_t> pcid_pml4_map; //ampping between pcid and pml4 base address
map<uint64_t,pair<uint16_t,uint64_t>> inverse_table; //PFN,pcid,VPN
map<uint64_t,pair<bool,bool>> nru_state; //nru maintained for the corresponding dram pages,(PFN,R,M)
vector<uint64_t> nru_pages; //stores only the PFN where R and M bit are 0
uint8_t page_swap = 0; //swap occured or not 




void record_roi_stats(uint32_t cpu, CACHE *cache)
{
    for (uint32_t i=0; i<NUM_TYPES; i++) {
        cache->roi_access[cpu][i] = cache->sim_access[cpu][i];
        cache->roi_hit[cpu][i] = cache->sim_hit[cpu][i];
        cache->roi_miss[cpu][i] = cache->sim_miss[cpu][i];
    }
}

void print_roi_stats(uint32_t cpu, CACHE *cache)
{
    uint64_t TOTAL_ACCESS = 0, TOTAL_HIT = 0, TOTAL_MISS = 0;

    for (uint32_t i=0; i<NUM_TYPES; i++) {
        TOTAL_ACCESS += cache->roi_access[cpu][i];
        TOTAL_HIT += cache->roi_hit[cpu][i];
        TOTAL_MISS += cache->roi_miss[cpu][i];
    }

    cout << cache->NAME;
    cout << " TOTAL     ACCESS: " << setw(10) << TOTAL_ACCESS << "  HIT: " << setw(10) << TOTAL_HIT << "  MISS: " << setw(10) << TOTAL_MISS << endl;

    cout << cache->NAME;
    cout << " LOAD      ACCESS: " << setw(10) << cache->roi_access[cpu][0] << "  HIT: " << setw(10) << cache->roi_hit[cpu][0] << "  MISS: " << setw(10) << cache->roi_miss[cpu][0] << endl;

    cout << cache->NAME;
    cout << " RFO       ACCESS: " << setw(10) << cache->roi_access[cpu][1] << "  HIT: " << setw(10) << cache->roi_hit[cpu][1] << "  MISS: " << setw(10) << cache->roi_miss[cpu][1] << endl;

    cout << cache->NAME;
    cout << " PREFETCH  ACCESS: " << setw(10) << cache->roi_access[cpu][2] << "  HIT: " << setw(10) << cache->roi_hit[cpu][2] << "  MISS: " << setw(10) << cache->roi_miss[cpu][2] << endl;

    cout << cache->NAME;
    cout << " WRITEBACK ACCESS: " << setw(10) << cache->roi_access[cpu][3] << "  HIT: " << setw(10) << cache->roi_hit[cpu][3] << "  MISS: " << setw(10) << cache->roi_miss[cpu][3] << endl;

    cout << cache->NAME;
    cout << " PREFETCH  REQUESTED: " << setw(10) << cache->pf_requested << "  ISSUED: " << setw(10) << cache->pf_issued;
    cout << "  USEFUL: " << setw(10) << cache->pf_useful << "  USELESS: " << setw(10) << cache->pf_useless << endl;
}

void print_sim_stats(uint32_t cpu, CACHE *cache)
{
    uint64_t TOTAL_ACCESS = 0, TOTAL_HIT = 0, TOTAL_MISS = 0;

    for (uint32_t i=0; i<NUM_TYPES; i++) {
        TOTAL_ACCESS += cache->sim_access[cpu][i];
        TOTAL_HIT += cache->sim_hit[cpu][i];
        TOTAL_MISS += cache->sim_miss[cpu][i];
    }

    cout << cache->NAME;
    cout << " TOTAL     ACCESS: " << setw(10) << TOTAL_ACCESS << "  HIT: " << setw(10) << TOTAL_HIT << "  MISS: " << setw(10) << TOTAL_MISS << endl;

    cout << cache->NAME;
    cout << " LOAD      ACCESS: " << setw(10) << cache->sim_access[cpu][0] << "  HIT: " << setw(10) << cache->sim_hit[cpu][0] << "  MISS: " << setw(10) << cache->sim_miss[cpu][0] << endl;

    cout << cache->NAME;
    cout << " RFO       ACCESS: " << setw(10) << cache->sim_access[cpu][1] << "  HIT: " << setw(10) << cache->sim_hit[cpu][1] << "  MISS: " << setw(10) << cache->sim_miss[cpu][1] << endl;

    cout << cache->NAME;
    cout << " PREFETCH  ACCESS: " << setw(10) << cache->sim_access[cpu][2] << "  HIT: " << setw(10) << cache->sim_hit[cpu][2] << "  MISS: " << setw(10) << cache->sim_miss[cpu][2] << endl;

    cout << cache->NAME;
    cout << " WRITEBACK ACCESS: " << setw(10) << cache->sim_access[cpu][3] << "  HIT: " << setw(10) << cache->sim_hit[cpu][3] << "  MISS: " << setw(10) << cache->sim_miss[cpu][3] << endl;
}

void print_branch_stats()
{
    for (uint32_t i=0; i<NUM_CPUS; i++) {
        cout << endl << "CPU " << i << " Branch Prediction Accuracy: ";
        cout << (100.0*(ooo_cpu[i].num_branch - ooo_cpu[i].branch_mispredictions)) / ooo_cpu[i].num_branch;
        cout << "% MPKI: " << (1000.0*ooo_cpu[i].branch_mispredictions)/(ooo_cpu[i].num_retired - ooo_cpu[i].warmup_instructions) << endl;
    }
}

void print_dram_stats()
{
    cout << endl;
    cout << "DRAM Statistics" << endl;
    for (uint32_t i=0; i<DRAM_CHANNELS; i++) {
        cout << " CHANNEL " << i << endl;
        cout << " RQ ROW_BUFFER_HIT: " << setw(10) << uncore.DRAM.RQ[i].ROW_BUFFER_HIT << "  ROW_BUFFER_MISS: " << setw(10) << uncore.DRAM.RQ[i].ROW_BUFFER_MISS << endl;
        cout << " DBUS_CONGESTED: " << setw(10) << uncore.DRAM.dbus_congested[NUM_TYPES][NUM_TYPES] << endl; 
        cout << " WQ ROW_BUFFER_HIT: " << setw(10) << uncore.DRAM.WQ[i].ROW_BUFFER_HIT << "  ROW_BUFFER_MISS: " << setw(10) << uncore.DRAM.WQ[i].ROW_BUFFER_MISS;
        cout << "  FULL: " << setw(10) << uncore.DRAM.WQ[i].FULL << endl; 
        cout << endl;
    }

    uint64_t total_congested_cycle = 0;
    for (uint32_t i=0; i<DRAM_CHANNELS; i++)
        total_congested_cycle += uncore.DRAM.dbus_cycle_congested[i];
    cout << " AVG_CONGESTED_CYCLE: " << (total_congested_cycle / uncore.DRAM.dbus_congested[NUM_TYPES][NUM_TYPES]) << endl;
}

void reset_cache_stats(uint32_t cpu, CACHE *cache)
{
    for (uint32_t i=0; i<NUM_TYPES; i++) {
        cache->ACCESS[i] = 0;
        cache->HIT[i] = 0;
        cache->MISS[i] = 0;
        cache->MSHR_MERGED[i] = 0;
        cache->STALL[i] = 0;

        cache->sim_access[cpu][i] = 0;
        cache->sim_hit[cpu][i] = 0;
        cache->sim_miss[cpu][i] = 0;
    }

    cache->RQ.ACCESS = 0;
    cache->RQ.MERGED = 0;
    cache->RQ.TO_CACHE = 0;

    cache->WQ.ACCESS = 0;
    cache->WQ.MERGED = 0;
    cache->WQ.TO_CACHE = 0;
    cache->WQ.FORWARD = 0;
    cache->WQ.FULL = 0;
}

void finish_warmup()
{
    uint64_t elapsed_second = (uint64_t)(time(NULL) - start_time),
             elapsed_minute = elapsed_second / 60,
             elapsed_hour = elapsed_minute / 60;
    elapsed_minute -= elapsed_hour*60;
    elapsed_second -= (elapsed_hour*3600 + elapsed_minute*60);

    // reset core latency
    SCHEDULING_LATENCY = 6;
    EXEC_LATENCY = 1;
    PAGE_TABLE_LATENCY = 100;
    SWAP_LATENCY = 100000;

    cout << endl;
    for (uint32_t i=0; i<NUM_CPUS; i++) {
        cout << "Warmup complete CPU " << i << " instructions: " << ooo_cpu[i].num_retired << " cycles: " << current_core_cycle[i];
        cout << " (Simulation time: " << elapsed_hour << " hr " << elapsed_minute << " min " << elapsed_second << " sec) " << endl;

        ooo_cpu[i].begin_sim_cycle = current_core_cycle[i]; 
        ooo_cpu[i].begin_sim_instr = ooo_cpu[i].num_retired;

        // reset branch stats
        ooo_cpu[i].num_branch = 0;
        ooo_cpu[i].branch_mispredictions = 0;

        reset_cache_stats(i, &ooo_cpu[i].L1I);
        reset_cache_stats(i, &ooo_cpu[i].L1D);
        reset_cache_stats(i, &ooo_cpu[i].L2C);
        reset_cache_stats(i, &uncore.LLC);
    }
    cout << endl;

    // reset DRAM stats
    for (uint32_t i=0; i<DRAM_CHANNELS; i++) {
        uncore.DRAM.RQ[i].ROW_BUFFER_HIT = 0;
        uncore.DRAM.RQ[i].ROW_BUFFER_MISS = 0;
        uncore.DRAM.WQ[i].ROW_BUFFER_HIT = 0;
        uncore.DRAM.WQ[i].ROW_BUFFER_MISS = 0;
    }

    // set actual cache latency
    for (uint32_t i=0; i<NUM_CPUS; i++) {
        ooo_cpu[i].ITLB.LATENCY = ITLB_LATENCY;
        ooo_cpu[i].DTLB.LATENCY = DTLB_LATENCY;
        ooo_cpu[i].STLB.LATENCY = STLB_LATENCY;
        ooo_cpu[i].L1I.LATENCY  = L1I_LATENCY;
        ooo_cpu[i].L1D.LATENCY  = L1D_LATENCY;
        ooo_cpu[i].L2C.LATENCY  = L2C_LATENCY;
    }
    uncore.LLC.LATENCY = LLC_LATENCY;
}

void print_deadlock(uint32_t i)
{
    cout << "DEADLOCK! CPU " << i << " instr_id: " << ooo_cpu[i].ROB.entry[ooo_cpu[i].ROB.head].instr_id;
    cout << " translated: " << +ooo_cpu[i].ROB.entry[ooo_cpu[i].ROB.head].translated;
    cout << " fetched: " << +ooo_cpu[i].ROB.entry[ooo_cpu[i].ROB.head].fetched;
    cout << " scheduled: " << +ooo_cpu[i].ROB.entry[ooo_cpu[i].ROB.head].scheduled;
    cout << " executed: " << +ooo_cpu[i].ROB.entry[ooo_cpu[i].ROB.head].executed;
    cout << " is_memory: " << +ooo_cpu[i].ROB.entry[ooo_cpu[i].ROB.head].is_memory;
    cout << " event: " << ooo_cpu[i].ROB.entry[ooo_cpu[i].ROB.head].event_cycle;
    cout << " current: " << current_core_cycle[i] << endl;

    // print LQ entry
    cout << endl << "Load Queue Entry" << endl;
    for (uint32_t j=0; j<LQ_SIZE; j++) {
        cout << "[LQ] entry: " << j << " instr_id: " << ooo_cpu[i].LQ.entry[j].instr_id << " address: " << hex << ooo_cpu[i].LQ.entry[j].physical_address << dec << " translated: " << +ooo_cpu[i].LQ.entry[j].translated << " fetched: " << +ooo_cpu[i].LQ.entry[i].fetched << endl;
    }

    // print SQ entry
    cout << endl << "Store Queue Entry" << endl;
    for (uint32_t j=0; j<SQ_SIZE; j++) {
        cout << "[SQ] entry: " << j << " instr_id: " << ooo_cpu[i].SQ.entry[j].instr_id << " address: " << hex << ooo_cpu[i].SQ.entry[j].physical_address << dec << " translated: " << +ooo_cpu[i].SQ.entry[j].translated << " fetched: " << +ooo_cpu[i].SQ.entry[i].fetched << endl;
    }

    // print L1D MSHR entry
    PACKET_QUEUE *queue;
    queue = &ooo_cpu[i].L1D.MSHR;
    cout << endl << queue->NAME << " Entry" << endl;
    for (uint32_t j=0; j<queue->SIZE; j++) {
        cout << "[" << queue->NAME << "] entry: " << j << " instr_id: " << queue->entry[j].instr_id << " rob_index: " << queue->entry[j].rob_index;
        cout << " address: " << hex << queue->entry[j].address << " full_addr: " << queue->entry[j].full_addr << dec << " type: " << +queue->entry[j].type;
        cout << " fill_level: " << queue->entry[j].fill_level << " lq_index: " << queue->entry[j].lq_index << " sq_index: " << queue->entry[j].sq_index << endl; 
    }

    assert(0);
}

void signal_handler(int signal) 
{
	cout << "Caught signal: " << signal << endl;
	exit(1);
}

// log base 2 function from efectiu
int lg2(int n)
{
    int i, m = n, c = -1;
    for (i=0; m; i++) {
        m /= 2;
        c++;
    }
    return c;
}

uint64_t rotl64 (uint64_t n, unsigned int c)
{
    const unsigned int mask = (CHAR_BIT*sizeof(n)-1);

    assert ( (c<=mask) &&"rotate by type width or more");
    c &= mask;  // avoid undef behaviour with NDEBUG.  0 overhead for most types / compilers
    return (n<<c) | (n>>( (-c)&mask ));
}

uint64_t rotr64 (uint64_t n, unsigned int c)
{
    const unsigned int mask = (CHAR_BIT*sizeof(n)-1);

    assert ( (c<=mask) &&"rotate by type width or more");
    c &= mask;  // avoid undef behaviour with NDEBUG.  0 overhead for most types / compilers
    return (n>>c) | (n<<( (-c)&mask ));
}

RANDOM champsim_rand(champsim_seed);


void delete_table(uint64_t *p,uint8_t level){  
    uint64_t i;
    uint64_t next_pt_base = UINT64_MAX;
    for(i = 0;i<NUM_ENTRIES;i++){
        uint64_t entry = *(p+i); 
        bool p = entry & 1;
        if(p){
            if(level<=3){
                next_pt_base = (entry >> (P) & ((1 << M) - 1));

#ifdef SANITY_CHECK
    if(next_pt_base == UINT64_MAX){
        assert(0);
    }
#endif

            delete_table((uint64_t*)next_pt_base,level+1);
            }
        } 
            
    }
}

void delete_allocated_page_tables(){
    map<uint16_t,uint64_t>::iterator it;
    for(it = pcid_pml4_map.begin(); it!=pcid_pml4_map.end();it++){
        delete_table((uint64_t*)it->second,1);
    }
}

void update_NRU_state(uint64_t PFN,uint8_t type){
    map<uint64_t,pair<bool,bool>>::iterator it = nru_state.find(PFN);
    bool Referenced = 0;
    bool Modified = 0;
    
    if(type == RFO){
        Modified = 1;
        Referenced = 1;
    }
    if(type == LOAD){
        Referenced = 1;
    }
    
    if(it == nru_state.end()){
        nru_state.insert(make_pair(PFN,make_pair(Referenced,Modified)));
    }
    else{
        if((it->second.first == 0 && it->second.second == 0) && (Referenced == 1 || Modified == 1)){
            uint64_t i;
            for(i=0;i<nru_pages.size();i++){
                if(nru_pages[i] == PFN){
                    break;
                }
            }

#ifdef SANITY_CHECK
     if(i == nru_pages.size()){
        assert(0);
     }       
#endif

            if(i<nru_pages.size()){
                nru_pages.erase(nru_pages.begin()+i);
            }
        }
        it->second.first = Referenced;
        it->second.second = it->second.second | Modified;
    }
}

uint8_t update_page_table(uint64_t* pt_base,uint64_t pcid,uint64_t VPN,uint8_t level,uint32_t cpu){

    //extract offset for that level
    //level 1: 27 to 35
    //level 2: 18 to 26
    //level 3:  9 to 17
    //level 4:  0 to 8
    uint64_t offset;
    uint8_t next_pt_status = 0;
    uint8_t status = 0;

    if(level<=3){
        offset = ((VPN >> ((4-level)*9 )) & ((1 << 9) -1));
    
    }
    else {
        offset = (VPN & ((1 << 9) - 1));
    }
    //current page table entry 
    uint64_t entry = *(pt_base+offset); 
    
    //check of ps = 0(page size) and p = 1(present) 
    bool p = entry & 1 ;
    bool ps  = (entry >> 7) & 1; 

#ifdef SANITY_CHECK
            if (!p || (ps && level<4))
                assert(0);
            if((!p || !ps) && level==4)
                assert(0);
#endif

    if(level<4){
        uint64_t next_pt_base = ((entry >> (P)) & ((1 << M) - 1));
        next_pt_status = update_page_table((uint64_t*)next_pt_base,pcid,VPN,level+1,cpu);
        if(next_pt_status == 1){
            {   
                uint64_t address = VPN >> ((4-level)*9);
                if(level == 1){
                    ooo_cpu[cpu].PML4C.invalidate_entry(address,pcid);
                }
                else if(level == 2){
                    ooo_cpu[cpu].PDPTC.invalidate_entry(address,pcid);
                }
                else if(level == 3){
                    ooo_cpu[cpu].PDC.invalidate_entry(address,pcid);
                }
                delete[] (uint64_t*)(next_pt_base);
                *(pt_base+offset) = (uint64_t)0;
            }   
        }       
    }
    if(level==4){   
        *(pt_base+offset) = (uint64_t)0;  
    }
    if(level != 1){
        uint64_t i;
        for(i=0;i<NUM_ENTRIES;i++){
            p = entry & 1;
            if(p)
                break;
        }
        if(i == NUM_ENTRIES){
            status = 1;
        }
    }
    return status;
} 

uint64_t allocate_page(uint32_t cpu){
    uint64_t NRU_ppage = UINT64_MAX;

    //nru_pages maintained for faster access to class 1 Physical frames/pages
    if(nru_pages.size()){
        NRU_ppage = nru_pages[0]; // implement it
    }
    else {
        map<uint64_t,pair<bool,bool>>::iterator it;
        //class 2 to 4
        uint64_t NRU_ppage_class[3]={UINT64_MAX,UINT64_MAX,UINT64_MAX};
        for(it = nru_state.begin();it!=nru_state.end();it++)   {
            if(it->second.first == 0 && it->second.second == 1 && NRU_ppage_class[0]==UINT64_MAX){
                NRU_ppage_class[0] = it->first;
                break;
            }
            else if(it->second.first == 1 && it->second.second == 0 && NRU_ppage_class[1]==UINT64_MAX){
                NRU_ppage_class[1] = it->first;                                           
            }
            else if(it->second.first == 1 && it->second.second == 1 && NRU_ppage_class[2]==UINT64_MAX) {
                NRU_ppage_class[2] = it->first;           
            }
        }
        for(int i=0;i<3;i++){
            if(NRU_ppage_class[i]!=UINT64_MAX){
                NRU_ppage = NRU_ppage_class[i];
                break;
            }
        }
    }
        
#ifdef SANITY_CHECK
        if (NRU_ppage == UINT64_MAX)
            assert(0);
#endif

    //invalidate the page table for the NRU page mapping for that VPN with that PCID 
    map<uint64_t,pair<uint16_t,uint64_t>>::iterator it1 = inverse_table.find(NRU_ppage);

#ifdef SANITY_CHECK
    if (it1 == inverse_table.end())
        assert(0);
#endif

    uint64_t NRU_pcid = it1->second.first;
    uint64_t NRU_VPN = it1->second.second;     
    map<uint16_t,uint64_t>::iterator it2 = pcid_pml4_map.find(NRU_pcid);

#ifdef SANITY_CHECK
   if (it2 == pcid_pml4_map.end())
        assert(0);
#endif

    uint64_t* NRU_PML = (uint64_t*)it2->second;
    uint8_t level = 1;
    uint8_t status = update_page_table(NRU_PML,NRU_pcid,NRU_VPN,level,cpu);
#ifdef SANITY_CHECK
    if(status == 1)
       assert(0);
#endif
    //invalidating cache,TLB,MMU Cache
    // invalidate corresponding NRU vpage and NRU ppage from the cache hierarchy
    ooo_cpu[cpu].ITLB.invalidate_entry(NRU_VPN,NRU_pcid);
    ooo_cpu[cpu].DTLB.invalidate_entry(NRU_VPN,NRU_pcid);
    ooo_cpu[cpu].STLB.invalidate_entry(NRU_VPN,NRU_pcid);
    for (uint32_t i=0; i<BLOCK_SIZE; i++) {
        uint64_t cl_addr = (NRU_ppage << 6) | i;
        ooo_cpu[cpu].L1I.invalidate_entry(cl_addr,UINT16_MAX);
        ooo_cpu[cpu].L1D.invalidate_entry(cl_addr,UINT16_MAX);
        ooo_cpu[cpu].L2C.invalidate_entry(cl_addr,UINT16_MAX);
        uncore.LLC.invalidate_entry(cl_addr,UINT16_MAX);
    }
    return NRU_ppage;
}

//where nru_state and nru_pages updated

uint64_t page_table_walk(uint64_t  *pt_base,uint64_t va_addr,uint8_t level,uint16_t pcid,uint32_t cpu) {
    
    uint64_t PFN = UINT64_MAX;
    uint64_t *PT;
    //extract offset for that level
    //level 1: 39 to 47
    //level 2: 30 to 38
    //level 3: 21 to 29
    //level 4: 12 to 20
    uint64_t offset = (va_addr >> ((4-level)*9+12)) & ((1 << 9) -1);
    
    //current page table entry 
    uint64_t entry = *(pt_base+offset); 
    
    //check of ps = 0(page size) and p = 1(present) 
    bool p = entry & 1 ;
    bool ps  = (entry >> 7) & 1;
    
    if(level<=3){   
        bool to_insert_mmu_cache = 1;
        if(!p){
            to_insert_mmu_cache = 0;
            PT = new uint64_t[NUM_ENTRIES]();
            
#ifdef SANITY_CHECK
    if(PT == NULL){
        assert(0);
    }    
#endif          
    
            entry = entry | ((uint64_t)(&PT[0]) << (P)); //setting next pointer base
            entry = entry | 1; //setting P bit
            entry = entry | (1 << 5); //setting accessed bit
            *(pt_base+offset) = entry;
            p = entry & 1; 
        }
        if(!ps && p){
            uint64_t next_pt_base = (entry >> (P)) & ((1 << M) - 1);
            PFN = page_table_walk((uint64_t*)next_pt_base,va_addr,level+1,pcid,cpu);
            if(to_insert_mmu_cache){
                uint64_t address = va_addr >> ((4-level)*9+12);
                
                PACKET mmu_cache_packet;
                mmu_cache_packet.data = next_pt_base;
                mmu_cache_packet.address = address;
                mmu_cache_packet.cpu = cpu;
                mmu_cache_packet.asid[0] = cpu;
                mmu_cache_packet.asid[1] = cpu;

                uint32_t set, way;

                if(level == 1){
                    set = ooo_cpu[cpu].PML4C.get_set(mmu_cache_packet.address);
                    way = ooo_cpu[cpu].PML4C.find_victim(cpu, 0, set, NULL, 0, 0, 0);
                    ooo_cpu[cpu].PML4C.update_replacement_state(cpu, set, way, 0, 0, 0, 0, 0);
                    ooo_cpu[cpu].PML4C.fill_cache(set, way, &mmu_cache_packet);
                }
                else if(level == 2){
                    set = ooo_cpu[cpu].PDPTC.get_set(mmu_cache_packet.address);
                    way = ooo_cpu[cpu].PDPTC.find_victim(cpu, 0, set, NULL, 0, 0, 0);
                    ooo_cpu[cpu].PDPTC.update_replacement_state(cpu, set, way, 0, 0, 0, 0, 0);
                    ooo_cpu[cpu].PDPTC.fill_cache(set, way, &mmu_cache_packet);
                }
                else if(level == 3){
                    set = ooo_cpu[cpu].PDC.get_set(mmu_cache_packet.address);
                    way = ooo_cpu[cpu].PDC.find_victim(cpu, 0, set, NULL, 0, 0, 0);
                    ooo_cpu[cpu].PDC.update_replacement_state(cpu, set, way, 0, 0, 0, 0, 0);
                    ooo_cpu[cpu].PDC.fill_cache(set, way, &mmu_cache_packet);    
                }
            }
                        
        }   
    }   
    else if(level == 4){
        if(!p){
            if(allocated_pages >= DRAM_PAGES){
                PFN = allocate_page(cpu);
                // swap complete , upgrade major fault in page_table_walk()
                page_swap = 1;
            }
            else{
                //finding empty DRAM page
                // smart random number generator
                uint64_t random_ppage;
                uint8_t fragmented = 0;
                if (num_adjacent_page > 0)
                    random_ppage = ++previous_ppage;
                else {
                    random_ppage = champsim_rand.draw_rand();
                    fragmented = 1;
                }

                // encoding cpu number 
                // this allows ChampSim to run homogeneous multi-programmed workloads without VA => PA aliasing
                // (e.g., cpu0: astar  cpu1: astar  cpu2: astar  cpu3: astar...)
                //random_ppage &= (~((NUM_CPUS-1)<< (32-LOG2_PAGE_SIZE)));
                //random_ppage |= (cpu<<(32-LOG2_PAGE_SIZE)); 
                map<uint64_t,pair<uint16_t,uint64_t>>::iterator ppage_check;
                while (1) { // try to find an empty physical page number
                    ppage_check = inverse_table.find(random_ppage); // check if this page can be allocated 
                    if (ppage_check != inverse_table.end()) { // random_ppage is not available
                        
                        if (num_adjacent_page > 0)
                            fragmented = 1;

                        // try one more time
                        random_ppage = champsim_rand.draw_rand();
                        
                        // encoding cpu number 
                        //random_ppage &= (~((NUM_CPUS-1)<<(32-LOG2_PAGE_SIZE)));
                        //random_ppage |= (cpu<<(32-LOG2_PAGE_SIZE)); 
                    }
                    else
                        break;
                }
                // insert translation to page tables of process
                PFN = random_ppage;
                previous_ppage = random_ppage;
                num_adjacent_page--;
                num_page[cpu]++;
                allocated_pages++;
                // try to allocate pages contiguously
                if (fragmented) {
                    num_adjacent_page = 1 << (rand() % 10);
                    /*DP ( if (warmup_complete[cpu]) {
                    cout << "Recalculate num_adjacent_page: " << num_adjacent_page << endl; });*/
                }
            }   
            #ifdef SANITY_CHECK
                if(PFN == UINT64_MAX)
                    assert(0);
            #endif
            //also update new VPN->PFN bonding, this is done in the page_table_walk
            entry = entry | 1; //setting P bit
            entry = entry | (1<<7);  //setting PS bit
            entry = entry | (PFN << P); //setting the PFN in entry
            entry = entry | (1 << 5); //setting accessed bit
            *(pt_base+offset) = entry;
            if (page_swap)
                major_fault[cpu]++;
            else
                minor_fault[cpu]++;
        }
        else if(p && ps){
            PFN = (entry >> (P) & ((1 << M) - 1));
        }   
    }
    return PFN; 
} 

uint64_t va_to_pa(uint32_t cpu, uint64_t va_addr, uint16_t pcid, uint8_t type)
{
#ifdef SANITY_CHECK
    if (va_addr == 0) 
        assert(0);
#endif
    //before going for page table walk check in mmu cache
    uint8_t level; 
    bool in_mmu_cache = 0;
    uint32_t set; 
    int way;
    uint64_t address;
    uint64_t pt_base;
    uint64_t PFN;
    uint64_t pa = 0;
    PACKET mmu_cache_packet;

    mmu_cache_packet.cpu = cpu;
    mmu_cache_packet.asid[0] = cpu;
    mmu_cache_packet.asid[1] = cpu;
    
    //checking in PDC
    level = 3;
    address = va_addr >> ((4-level)*9+12);
    mmu_cache_packet.address = address;
    set = ooo_cpu[cpu].PDC.get_set(address);
    way = ooo_cpu[cpu].PDC.check_hit(&mmu_cache_packet);         
    if (way >= 0) { // read hit
        in_mmu_cache = 1;
        pt_base = ooo_cpu[cpu].PDC.block[set][way].data;
        PFN = page_table_walk((uint64_t*)pt_base,va_addr,level+1,pcid,cpu);
        ooo_cpu[cpu].PDC.update_replacement_state(cpu, set, way, 0, 0, 0, 0, 0);    
    }
    //checking in PDPTC
    else{

#ifdef SANITY_CHECK
    if(in_mmu_cache){
        assert(0);
    }    
#endif
        
        level--;
        address = va_addr >> ((4-level)*9+12);
        mmu_cache_packet.address = address;
        set = ooo_cpu[cpu].PDPTC.get_set(address);
        way = ooo_cpu[cpu].PDPTC.check_hit(&mmu_cache_packet);
        if (way >= 0) { // read hit
            in_mmu_cache = 1;
            pt_base = ooo_cpu[cpu].PDPTC.block[set][way].data;
            PFN = page_table_walk((uint64_t*)pt_base,va_addr,level+1,pcid,cpu); 
            ooo_cpu[cpu].PDPTC.update_replacement_state(cpu, set, way, 0, 0, 0, 0, 0);   
        }
//checking in PTC    
        else{

#ifdef SANITY_CHECK
    if(in_mmu_cache){
        assert(0);
    }    
#endif

            level--;
            address = va_addr >> ((4-level)*9+12);
            mmu_cache_packet.address = address;
            set = ooo_cpu[cpu].PML4C.get_set(address);
            way = ooo_cpu[cpu].PML4C.check_hit(&mmu_cache_packet);
            if (way >= 0) { // read hit
                in_mmu_cache = 1;
                pt_base = ooo_cpu[cpu].PML4C.block[set][way].data;
                PFN = page_table_walk((uint64_t*)pt_base,va_addr,level+1,pcid,cpu);    
                ooo_cpu[cpu].PML4C.update_replacement_state(cpu, set, way, 0, 0, 0, 0, 0);
            }   
        }
    }

    if(in_mmu_cache == 0){

        page_swap = 0;
        uint64_t *PML;
        map<uint16_t,uint64_t>::iterator it;

        //check pml4 exists, if not exists create one and map it with corresponding pcid
        it = pcid_pml4_map.find(pcid);

        if(it == pcid_pml4_map.end()){
            PML = new uint64_t[NUM_ENTRIES]();

#ifdef SANITY_CHECK
    if(PML == NULL){
        assert(0);
    }    
#endif

            pcid_pml4_map.insert(make_pair(pcid,(uint64_t)&PML[0]));       
        } 
        it = pcid_pml4_map.find(pcid);

#ifdef SANITY_CHECK
    if(it == pcid_pml4_map.end()){
        assert(0);
    }
#endif        

        // page table walk using base address,VA,current level in page table
        PFN = page_table_walk((uint64_t*)it->second,va_addr,level,pcid,cpu);

        // update inverse table with new PA => VA mapping in main() and update NRU state in main()         
        map<uint64_t,pair<uint16_t,uint64_t>>::iterator it1 = inverse_table.find(PFN);
        if (it1 == inverse_table.end()){
            inverse_table.insert(make_pair(PFN,make_pair(pcid,va_addr >> 12)));
        }
        else{ 
            it1->second.first = pcid; //setting the pcid
            it1->second.second = va_addr >> 12; //setting the VPN
        }
        
        uint64_t retry_PFN = page_table_walk((uint64_t*)it->second,va_addr,level,pcid,cpu);
        it1 = inverse_table.find(retry_PFN);

#ifdef SANITY_CHECK
    if (it1 == inverse_table.end())
        assert(0);
    if(retry_PFN != PFN){
        assert(0);
    }
#endif

    }    
    
    //update NRU state
    update_NRU_state(PFN,type);

    uint64_t voffset = va_addr & ((1 << LOG2_PAGE_SIZE)-1);
    pa = PFN << LOG2_PAGE_SIZE;
    pa |= voffset;
    if (page_swap)
       stall_cycle[cpu] = current_core_cycle[cpu] + SWAP_LATENCY;
    else
        stall_cycle[cpu] = current_core_cycle[cpu] + PAGE_TABLE_LATENCY;
    
    return pa;
}

int main(int argc, char** argv)
{
	// interrupt signal hanlder
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = signal_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

    cout << endl << "*** ChampSim Multicore Out-of-Order Simulator ***" << endl << endl;

    // initialize knobs
    uint8_t show_heartbeat = 1;

    uint32_t seed_number = 0;

    // check to see if knobs changed using getopt_long()
    int c;
    while (1) {
        static struct option long_options[] =
        {
            {"warmup_instructions", required_argument, 0, 'w'},
            {"simulation_instructions", required_argument, 0, 'i'},
            {"hide_heartbeat", no_argument, 0, 'h'},
            {"cloudsuite", no_argument, 0, 'c'},
            {"low_bandwidth",  no_argument, 0, 'b'},
            {"traces",  no_argument, 0, 't'},
            {0, 0, 0, 0}      
        };

        int option_index = 0;

        c = getopt_long_only(argc, argv, "wihsb", long_options, &option_index);

        // no more option characters
        if (c == -1)
            break;

        int traces_encountered = 0;

        switch(c) {
            case 'w':
                warmup_instructions = atol(optarg);
                break;
            case 'i':
                simulation_instructions = atol(optarg);
                break;
            case 'h':
                show_heartbeat = 0;
                break;
            case 'c':
                knob_cloudsuite = 1;
                MAX_INSTR_DESTINATIONS = NUM_INSTR_DESTINATIONS_SPARC;
                break;
            case 'b':
                knob_low_bandwidth = 1;
                break;
            case 't':
                traces_encountered = 1;
                break;
            default:
                abort();
        }

        if (traces_encountered == 1)
            break;
    }

    // consequences of knobs
    cout << "Warmup Instructions: " << warmup_instructions << endl;
    cout << "Simulation Instructions: " << simulation_instructions << endl;
    //cout << "Scramble Loads: " << (knob_scramble_loads ? "ture" : "false") << endl;
    cout << "Number of CPUs: " << NUM_CPUS << endl;
    cout << "LLC sets: " << LLC_SET << endl;
    cout << "LLC ways: " << LLC_WAY << endl;

    if (knob_low_bandwidth)
        DRAM_MTPS = 400;
    else
        DRAM_MTPS = 1600;

    // DRAM access latency
    tRP  = tRP_DRAM_CYCLE  * (CPU_FREQ / DRAM_IO_FREQ); 
    tRCD = tRCD_DRAM_CYCLE * (CPU_FREQ / DRAM_IO_FREQ); 
    tCAS = tCAS_DRAM_CYCLE * (CPU_FREQ / DRAM_IO_FREQ); 

    // default: 16 = (64 / 8) * (3200 / 1600)
    // it takes 16 CPU cycles to tranfser 64B cache block on a 8B (64-bit) bus 
    // note that dram burst length = BLOCK_SIZE/DRAM_CHANNEL_WIDTH
    DRAM_DBUS_RETURN_TIME = (BLOCK_SIZE / DRAM_CHANNEL_WIDTH) * (CPU_FREQ / DRAM_MTPS);

    printf("Off-chip DRAM Size: %u MB Channels: %u Width: %u-bit Data Rate: %u MT/s\n",
            DRAM_SIZE, DRAM_CHANNELS, 8*DRAM_CHANNEL_WIDTH, DRAM_MTPS);

    // end consequence of knobs

    // search through the argv for "-traces"
    int found_traces = 0;
    int count_traces = 0;
    cout << endl;
    for (int i=0; i<argc; i++) {
        if (found_traces) {
            printf("CPU %d runs %s\n", count_traces, argv[i]);

            sprintf(ooo_cpu[count_traces].trace_string, "%s", argv[i]);

            char *full_name = ooo_cpu[count_traces].trace_string,
                 *last_dot = strrchr(ooo_cpu[count_traces].trace_string, '.');

            if (full_name[last_dot - full_name + 1] == 'g') // gzip format
                sprintf(ooo_cpu[count_traces].gunzip_command, "gunzip -c %s", argv[i]);
            else if (full_name[last_dot - full_name + 1] == 'x') // xz
                sprintf(ooo_cpu[count_traces].gunzip_command, "xz -dc %s", argv[i]);
            else {
                cout << "ChampSim does not support traces other than gz or xz compression!" << endl; 
                assert(0);
            }

            char *pch[100];
            int count_str = 0;
            pch[0] = strtok (argv[i], " /,.-");
            while (pch[count_str] != NULL) {
                //printf ("%s %d\n", pch[count_str], count_str);
                count_str++;
                pch[count_str] = strtok (NULL, " /,.-");
            }

            //printf("max count_str: %d\n", count_str);
            //printf("application: %s\n", pch[count_str-3]);

            int j = 0;
            while (pch[count_str-3][j] != '\0') {
                seed_number += pch[count_str-3][j];
                //printf("%c %d %d\n", pch[count_str-3][j], j, seed_number);
                j++;
            }

            ooo_cpu[count_traces].trace_file = popen(ooo_cpu[count_traces].gunzip_command, "r");
            if (ooo_cpu[count_traces].trace_file == NULL) {
                printf("\n*** Trace file not found: %s ***\n\n", argv[i]);
                assert(0);
            }

            count_traces++;
            if (count_traces > NUM_CPUS) {
                printf("\n*** Too many traces for the configured number of cores ***\n\n");
                assert(0);
            }
        }
        else if(strcmp(argv[i],"-traces") == 0) {
            found_traces = 1;
        }
    }

    if (count_traces != NUM_CPUS) {
        printf("\n*** Not enough traces for the configured number of cores ***\n\n");
        assert(0);
    }
    // end trace file setup

    
    // TODO: can we initialize these variables from the class constructor?
    srand(seed_number);
    champsim_seed = seed_number;
    for (int i=0; i<NUM_CPUS; i++) {

        ooo_cpu[i].cpu = i; 
        ooo_cpu[i].warmup_instructions = warmup_instructions;
        ooo_cpu[i].simulation_instructions = simulation_instructions;
        ooo_cpu[i].begin_sim_cycle = 0; 
        ooo_cpu[i].begin_sim_instr = warmup_instructions;

        // ROB
        ooo_cpu[i].ROB.cpu = i;

        // BRANCH PREDICTOR
        ooo_cpu[i].initialize_branch_predictor();

        // TLBs
        ooo_cpu[i].ITLB.cpu = i;
        ooo_cpu[i].ITLB.cache_type = IS_ITLB;
        ooo_cpu[i].ITLB.fill_level = FILL_L1;
        ooo_cpu[i].ITLB.extra_interface = &ooo_cpu[i].L1I;
        ooo_cpu[i].ITLB.lower_level = &ooo_cpu[i].STLB; 

        ooo_cpu[i].DTLB.cpu = i;
        ooo_cpu[i].DTLB.cache_type = IS_DTLB;
        ooo_cpu[i].DTLB.MAX_READ = (2 > MAX_READ_PER_CYCLE) ? MAX_READ_PER_CYCLE : 2;
        ooo_cpu[i].DTLB.fill_level = FILL_L1;
        ooo_cpu[i].DTLB.extra_interface = &ooo_cpu[i].L1D;
        ooo_cpu[i].DTLB.lower_level = &ooo_cpu[i].STLB;

        ooo_cpu[i].STLB.cpu = i;
        ooo_cpu[i].STLB.cache_type = IS_STLB;
        ooo_cpu[i].STLB.fill_level = FILL_L2;
        ooo_cpu[i].STLB.upper_level_icache[i] = &ooo_cpu[i].ITLB;
        ooo_cpu[i].STLB.upper_level_dcache[i] = &ooo_cpu[i].DTLB;

        // PRIVATE CACHE
        ooo_cpu[i].L1I.cpu = i;
        ooo_cpu[i].L1I.cache_type = IS_L1I;
        ooo_cpu[i].L1I.MAX_READ = (FETCH_WIDTH > MAX_READ_PER_CYCLE) ? MAX_READ_PER_CYCLE : FETCH_WIDTH;
        ooo_cpu[i].L1I.fill_level = FILL_L1;
        ooo_cpu[i].L1I.lower_level = &ooo_cpu[i].L2C; 

        ooo_cpu[i].L1D.cpu = i;
        ooo_cpu[i].L1D.cache_type = IS_L1D;
        ooo_cpu[i].L1D.MAX_READ = (2 > MAX_READ_PER_CYCLE) ? MAX_READ_PER_CYCLE : 2;
        ooo_cpu[i].L1D.fill_level = FILL_L1;
        ooo_cpu[i].L1D.lower_level = &ooo_cpu[i].L2C; 
        ooo_cpu[i].L1D.l1d_prefetcher_initialize();

        ooo_cpu[i].L2C.cpu = i;
        ooo_cpu[i].L2C.cache_type = IS_L2C;
        ooo_cpu[i].L2C.fill_level = FILL_L2;
        ooo_cpu[i].L2C.upper_level_icache[i] = &ooo_cpu[i].L1I;
        ooo_cpu[i].L2C.upper_level_dcache[i] = &ooo_cpu[i].L1D;
        ooo_cpu[i].L2C.lower_level = &uncore.LLC;
        ooo_cpu[i].L2C.l2c_prefetcher_initialize();

        // SHARED CACHE
        uncore.LLC.cache_type = IS_LLC;
        uncore.LLC.fill_level = FILL_LLC;
        uncore.LLC.upper_level_icache[i] = &ooo_cpu[i].L2C;
        uncore.LLC.upper_level_dcache[i] = &ooo_cpu[i].L2C;
        uncore.LLC.lower_level = &uncore.DRAM;

        // OFF-CHIP DRAM
        uncore.DRAM.fill_level = FILL_DRAM;
        uncore.DRAM.upper_level_icache[i] = &uncore.LLC;
        uncore.DRAM.upper_level_dcache[i] = &uncore.LLC;
        for (uint32_t i=0; i<DRAM_CHANNELS; i++) {
            uncore.DRAM.RQ[i].is_RQ = 1;
            uncore.DRAM.WQ[i].is_WQ = 1;
        }

        warmup_complete[i] = 0;
        //all_warmup_complete = NUM_CPUS;
        simulation_complete[i] = 0;
        current_core_cycle[i] = 0;
        stall_cycle[i] = 0;
        
        previous_ppage = 0;
        num_adjacent_page = 0;
        num_cl[i] = 0;
        allocated_pages = 0;
        num_page[i] = 0;
        minor_fault[i] = 0;
        major_fault[i] = 0;
    }

    uncore.LLC.llc_initialize_replacement();

    // simulation entry point
    start_time = time(NULL);
    uint8_t run_simulation = 1;
    uint64_t cycle = 0;
    while (run_simulation) {

        uint64_t elapsed_second = (uint64_t)(time(NULL) - start_time),
                 elapsed_minute = elapsed_second / 60,
                 elapsed_hour = elapsed_minute / 60;
        elapsed_minute -= elapsed_hour*60;
        elapsed_second -= (elapsed_hour*3600 + elapsed_minute*60);
        
        cycle++;
        for (int i=0; i<NUM_CPUS; i++) {
            // proceed one cycle
            current_core_cycle[i]++;
            
            //Refreshing the R bit every 200th cycle
            if(cycle%200 == 0){
            	map<uint64_t,pair<bool,bool>>::iterator it;
            	for(it = nru_state.begin();it!=nru_state.end();it++){
            		it->second.first = 0;
            		if(it->second.first == 0 && it->second.second == 0){
            			nru_pages.push_back(it->first);
            		}
            	}
            }

            //cout << "Trying to process instr_id: " << ooo_cpu[i].instr_unique_id << " fetch_stall: " << +ooo_cpu[i].fetch_stall;
            //cout << " stall_cycle: " << stall_cycle[i] << " current: " << current_core_cycle[i] << endl;

            // core might be stalled due to page fault or branch misprediction
            if (stall_cycle[i] <= current_core_cycle[i]) {

                // fetch unit
                if (ooo_cpu[i].ROB.occupancy < ooo_cpu[i].ROB.SIZE) {
                    // handle branch
                    if (ooo_cpu[i].fetch_stall == 0) 
                        ooo_cpu[i].handle_branch();
                }

                // fetch
                ooo_cpu[i].fetch_instruction();


                // schedule (including decode latency)
                uint32_t schedule_index = ooo_cpu[i].ROB.next_schedule;
                if ((ooo_cpu[i].ROB.entry[schedule_index].scheduled == 0) && (ooo_cpu[i].ROB.entry[schedule_index].event_cycle <= current_core_cycle[i]))
                    ooo_cpu[i].schedule_instruction();

                // execute
                ooo_cpu[i].execute_instruction();

                // memory operation
                ooo_cpu[i].schedule_memory_instruction();
                ooo_cpu[i].execute_memory_instruction();

                // complete 
                ooo_cpu[i].update_rob();

                // retire
                if ((ooo_cpu[i].ROB.entry[ooo_cpu[i].ROB.head].executed == COMPLETED) && (ooo_cpu[i].ROB.entry[ooo_cpu[i].ROB.head].event_cycle <= current_core_cycle[i]))
                    ooo_cpu[i].retire_rob();
            }

            // heartbeat information
            if (show_heartbeat && (ooo_cpu[i].num_retired >= ooo_cpu[i].next_print_instruction)) {
                float cumulative_ipc;
                if (warmup_complete[i])
                    cumulative_ipc = (1.0*(ooo_cpu[i].num_retired - ooo_cpu[i].begin_sim_instr)) / (current_core_cycle[i] - ooo_cpu[i].begin_sim_cycle);
                else
                    cumulative_ipc = (1.0*ooo_cpu[i].num_retired) / current_core_cycle[i];
                float heartbeat_ipc = (1.0*ooo_cpu[i].num_retired - ooo_cpu[i].last_sim_instr) / (current_core_cycle[i] - ooo_cpu[i].last_sim_cycle);

                cout << "Heartbeat CPU " << i << " instructions: " << ooo_cpu[i].num_retired << " cycles: " << current_core_cycle[i];
                cout << " heartbeat IPC: " << heartbeat_ipc << " cumulative IPC: " << cumulative_ipc; 
                cout << " (Simulation time: " << elapsed_hour << " hr " << elapsed_minute << " min " << elapsed_second << " sec) " << endl;
                ooo_cpu[i].next_print_instruction += STAT_PRINTING_PERIOD;

                ooo_cpu[i].last_sim_instr = ooo_cpu[i].num_retired;
                ooo_cpu[i].last_sim_cycle = current_core_cycle[i];
            }

            // check for deadlock
            if (ooo_cpu[i].ROB.entry[ooo_cpu[i].ROB.head].ip && (ooo_cpu[i].ROB.entry[ooo_cpu[i].ROB.head].event_cycle + DEADLOCK_CYCLE) <= current_core_cycle[i])
                print_deadlock(i);

            // check for warmup
            // warmup complete
            if ((warmup_complete[i] == 0) && (ooo_cpu[i].num_retired > warmup_instructions)) {
                warmup_complete[i] = 1;
                all_warmup_complete++;
            }
            if (all_warmup_complete == NUM_CPUS) { // this part is called only once when all cores are warmed up
                all_warmup_complete++;
                finish_warmup();
            }

            /*
            if (all_warmup_complete == 0) { 
                all_warmup_complete = 1;
                finish_warmup();
            }
            if (ooo_cpu[1].num_retired > 0)
                warmup_complete[1] = 1;
            */
            
            // simulation complete
            if ((all_warmup_complete > NUM_CPUS) && (simulation_complete[i] == 0) && (ooo_cpu[i].num_retired >= (ooo_cpu[i].begin_sim_instr + ooo_cpu[i].simulation_instructions))) {
                simulation_complete[i] = 1;
                ooo_cpu[i].finish_sim_instr = ooo_cpu[i].num_retired - ooo_cpu[i].begin_sim_instr;
                ooo_cpu[i].finish_sim_cycle = current_core_cycle[i] - ooo_cpu[i].begin_sim_cycle;

                cout << "Finished CPU " << i << " instructions: " << ooo_cpu[i].finish_sim_instr << " cycles: " << ooo_cpu[i].finish_sim_cycle;
                cout << " cumulative IPC: " << ((float) ooo_cpu[i].finish_sim_instr / ooo_cpu[i].finish_sim_cycle);
                cout << " (Simulation time: " << elapsed_hour << " hr " << elapsed_minute << " min " << elapsed_second << " sec) " << endl;

                record_roi_stats(i, &ooo_cpu[i].L1D);
                record_roi_stats(i, &ooo_cpu[i].L1I);
                record_roi_stats(i, &ooo_cpu[i].L2C);
                record_roi_stats(i, &uncore.LLC);

                all_simulation_complete++;
            }

            if (all_simulation_complete == NUM_CPUS)
                run_simulation = 0;
        }

        // TODO: should it be backward?
        uncore.LLC.operate();
        uncore.DRAM.operate();

    }

    
#ifndef CRC2_COMPILE
    print_branch_stats();
#endif
    uint64_t elapsed_second = (uint64_t)(time(NULL) - start_time),
             elapsed_minute = elapsed_second / 60,
             elapsed_hour = elapsed_minute / 60;
    elapsed_minute -= elapsed_hour*60;
    elapsed_second -= (elapsed_hour*3600 + elapsed_minute*60);
    
    cout << endl << "ChampSim completed all CPUs" << endl;
    if (NUM_CPUS > 1) {
        cout << endl << "Total Simulation Statistics (not including warmup)" << endl;
        for (uint32_t i=0; i<NUM_CPUS; i++) {
            cout << endl << "CPU " << i << " cumulative IPC: " << (float) (ooo_cpu[i].num_retired - ooo_cpu[i].begin_sim_instr) / (current_core_cycle[i] - ooo_cpu[i].begin_sim_cycle); 
            cout << " instructions: " << ooo_cpu[i].num_retired - ooo_cpu[i].begin_sim_instr << " cycles: " << current_core_cycle[i] - ooo_cpu[i].begin_sim_cycle << endl;
#ifndef CRC2_COMPILE
            print_sim_stats(i, &ooo_cpu[i].L1D);
            print_sim_stats(i, &ooo_cpu[i].L1I);
            print_sim_stats(i, &ooo_cpu[i].L2C);
            ooo_cpu[i].L1D.l1d_prefetcher_final_stats();
            ooo_cpu[i].L2C.l2c_prefetcher_final_stats();
#endif
            print_sim_stats(i, &uncore.LLC);
        }
    }

    cout << endl << "Region of Interest Statistics" << endl;
    delete_allocated_page_tables();
    for (uint32_t i=0; i<NUM_CPUS; i++) {
        cout << endl << "CPU " << i << " cumulative IPC: " << ((float) ooo_cpu[i].finish_sim_instr / ooo_cpu[i].finish_sim_cycle); 
        cout << " instructions: " << ooo_cpu[i].finish_sim_instr << " cycles: " << ooo_cpu[i].finish_sim_cycle << endl;
#ifndef CRC2_COMPILE
        print_roi_stats(i, &ooo_cpu[i].L1D);
        print_roi_stats(i, &ooo_cpu[i].L1I);
        print_roi_stats(i, &ooo_cpu[i].L2C);
#endif
        print_roi_stats(i, &uncore.LLC);
        cout << "Major fault: " << major_fault[i] << " Minor fault: " << minor_fault[i] << endl;
    }

    for (uint32_t i=0; i<NUM_CPUS; i++) {
        ooo_cpu[i].L1D.l1d_prefetcher_final_stats();
        ooo_cpu[i].L2C.l2c_prefetcher_final_stats();
    }

#ifndef CRC2_COMPILE
    uncore.LLC.llc_replacement_final_stats();
    print_dram_stats();
#endif

    return 0;
}
