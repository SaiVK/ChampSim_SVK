// CACHE BLOCK
class BLOCK {
  public:
    uint8_t valid,
            prefetch,
            dirty,
            used;

    int delta,
        depth,
        signature,
        confidence;

    uint64_t address,
             full_addr,
             tag,
             data,
             cpu,
             instr_id;

    // replacement state
    uint32_t lru;

    BLOCK() {
        valid = 0;
        prefetch = 0;
        dirty = 0;
        used = 0;

        delta = 0;
        depth = 0;
        signature = 0;
        confidence = 0;

        address = 0;
        full_addr = 0;
        tag = 0;
        data = 0;
        cpu = 0;
        instr_id = 0;

        lru = 0;
    };
};


//page table walk
uint64_t va_to_pa(uint32_t cpu, uint64_t instr_id, uint64_t va, uint64_t unique_vpage)
{
#ifdef SANITY_CHECK
    if (va == 0) 
        assert(0);
#endif

    uint8_t  swap = 0;
    uint64_t high_bit_mask = rotr64(cpu, lg2(NUM_CPUS)),
             unique_va = va | high_bit_mask;
    //uint64_t vpage = unique_va >> LOG2_PAGE_SIZE,
    uint64_t vpage = unique_vpage | high_bit_mask,
             voffset = unique_va & ((1<<LOG2_PAGE_SIZE) - 1);

    // smart random number generator
    uint64_t random_ppage;

    map <uint64_t, uint64_t>::iterator pr = page_table.begin();
    map <uint64_t, uint64_t>::iterator ppage_check = inverse_table.begin();

    // check unique cache line footprint
    map <uint64_t, uint64_t>::iterator cl_check = unique_cl[cpu].find(unique_va >> LOG2_BLOCK_SIZE);
    if (cl_check == unique_cl[cpu].end()) { // we've never seen this cache line before
        unique_cl[cpu].insert(make_pair(unique_va >> LOG2_BLOCK_SIZE, 0));
        num_cl[cpu]++;
    }
    else
        cl_check->second++;


    pr = page_table.find(vpage);
    if (pr == page_table.end()) { // no VA => PA translation found 

        if (allocated_pages >= DRAM_PAGES) { // not enough memory

            // TODO: elaborate page replacement algorithm
            // here, ChampSim randomly selects a page that is not recently used and we only track 32K recently accessed pages
            uint8_t  found_NRU = 0;
            uint64_t NRU_vpage = 0; // implement it
            map <uint64_t, uint64_t>::iterator pr2 = recent_page.begin();
            for (pr = page_table.begin(); pr != page_table.end(); pr++) {

                NRU_vpage = pr->first;
                if (recent_page.find(NRU_vpage) == recent_page.end()) {
                    found_NRU = 1;
                    break;
                }
            }
#ifdef SANITY_CHECK
            if (found_NRU == 0)
                assert(0);

            if (pr == page_table.end())
                assert(0);
#endif
            DP ( if (warmup_complete[cpu]) {
            cout << "[SWAP] update page table NRU_vpage: " << hex << pr->first << " new_vpage: " << vpage << " ppage: " << pr->second << dec << endl; });

            // update page table with new VA => PA mapping
            // since we cannot change the key value already inserted in a map structure, we need to erase the old node and add a new node
            uint64_t mapped_ppage = pr->second;
            page_table.erase(pr);
            page_table.insert(make_pair(vpage, mapped_ppage));

            // update inverse table with new PA => VA mapping
            ppage_check = inverse_table.find(mapped_ppage);
#ifdef SANITY_CHECK
            if (ppage_check == inverse_table.end())
                assert(0);
#endif
            ppage_check->second = vpage;

            DP ( if (warmup_complete[cpu]) {
            cout << "[SWAP] update inverse table NRU_vpage: " << hex << NRU_vpage << " new_vpage: ";
            cout << ppage_check->second << " ppage: " << ppage_check->first << dec << endl; });

            // update page_queue
            page_queue.pop();
            page_queue.push(vpage);

            // invalidate corresponding vpage and ppage from the cache hierarchy
            ooo_cpu[cpu].ITLB.invalidate_entry(NRU_vpage);
            ooo_cpu[cpu].DTLB.invalidate_entry(NRU_vpage);
            ooo_cpu[cpu].STLB.invalidate_entry(NRU_vpage);
            for (uint32_t i=0; i<BLOCK_SIZE; i++) {
                uint64_t cl_addr = (mapped_ppage << 6) | i;
                ooo_cpu[cpu].L1I.invalidate_entry(cl_addr);
                ooo_cpu[cpu].L1D.invalidate_entry(cl_addr);
                ooo_cpu[cpu].L2C.invalidate_entry(cl_addr);
                uncore.LLC.invalidate_entry(cl_addr);
            }

            // swap complete
            swap = 1;
        } else {
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

            while (1) { // try to find an empty physical page number
                ppage_check = inverse_table.find(random_ppage); // check if this page can be allocated 
                if (ppage_check != inverse_table.end()) { // random_ppage is not available
                    DP ( if (warmup_complete[cpu]) {
                    cout << "vpage: " << hex << ppage_check->first << " is already mapped to ppage: " << random_ppage << dec << endl; }); 
                    
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

            // insert translation to page tables
            //printf("Insert  num_adjacent_page: %u  vpage: %lx  ppage: %lx\n", num_adjacent_page, vpage, random_ppage);
            page_table.insert(make_pair(vpage, random_ppage));
            inverse_table.insert(make_pair(random_ppage, vpage));
            page_queue.push(vpage);
            previous_ppage = random_ppage;
            num_adjacent_page--;
            num_page[cpu]++;
            allocated_pages++;

            // try to allocate pages contiguously
            if (fragmented) {
                num_adjacent_page = 1 << (rand() % 10);
                DP ( if (warmup_complete[cpu]) {
                cout << "Recalculate num_adjacent_page: " << num_adjacent_page << endl; });
            }
        }

        if (swap)
            major_fault[cpu]++;
        else
            minor_fault[cpu]++;
    }
    else {
        //printf("Found  vpage: %lx  random_ppage: %lx\n", vpage, pr->second);
    }

    pr = page_table.find(vpage);
#ifdef SANITY_CHECK
    if (pr == page_table.end())
        assert(0);
#endif
    uint64_t ppage = pr->second;

    uint64_t pa = ppage << LOG2_PAGE_SIZE;
    pa |= voffset;

    DP ( if (warmup_complete[cpu]) {
    cout << "[PAGE_TABLE] instr_id: " << instr_id << " vpage: " << hex << vpage;
    cout << " => ppage: " << (pa >> LOG2_PAGE_SIZE) << " vadress: " << unique_va << " paddress: " << pa << dec << endl; });

    if (swap)
        stall_cycle[cpu] = current_core_cycle[cpu] + SWAP_LATENCY;
    else
        stall_cycle[cpu] = current_core_cycle[cpu] + PAGE_TABLE_LATENCY;

    //cout << "cpu: " << cpu << " allocated unique_vpage: " << hex << unique_vpage << " to ppage: " << ppage << dec << endl;

    return pa;
}
