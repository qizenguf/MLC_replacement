/*
 * Copyright (c) 2013 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Erik Hallnor
 *          Ron Dreslinski
 */

/**
 * @file
 * Definitions of BaseTags.
 */

#include "mem/cache/tags/base.hh"

#include "cpu/smt.hh" //maxThreadsPerCPU
#include "mem/cache/base.hh"
#include "sim/sim_exit.hh"

using namespace std;

BaseTags::BaseTags(const Params *p)
    : ClockedObject(p), blkSize(p->block_size), size(p->size),
      accessLatency(p->hit_latency), cache(nullptr), warmupBound(0),
      warmedUp(false), numBlocks(0)
{
}

void
BaseTags::setCache(BaseCache *_cache)
{
    assert(!cache);
    cache = _cache;
}

void
BaseTags::regStats()
{
    ClockedObject::regStats();

    using namespace Stats;

    replacements
        .init(maxThreadsPerCPU)
        .name(name() + ".replacements")
        .desc("number of replacements")
        .flags(total)
        ;

    tagsInUse
        .name(name() + ".tagsinuse")
        .desc("Cycle average of tags in use")
        ;

    totalRefs
        .name(name() + ".total_refs")
        .desc("Total number of references to valid blocks.")
        ;

    sampledRefs
        .name(name() + ".sampled_refs")
        .desc("Sample count of references to valid blocks.")
        ;

    avgRefs
        .name(name() + ".avg_refs")
        .desc("Average number of references to valid blocks.")
        ;

    avgRefs = totalRefs/sampledRefs;

    warmupCycle
        .name(name() + ".warmup_cycle")
        .desc("Cycle when the warmup percentage was hit.")
        ;

    occupancies
        .init(cache->system->maxMasters())
        .name(name() + ".occ_blocks")
        .desc("Average occupied blocks per requestor")
        .flags(nozero | nonan)
        ;
    for (int i = 0; i < cache->system->maxMasters(); i++) {
        occupancies.subname(i, cache->system->getMasterName(i));
    }
    //add by qi 
    avgZT
		.init(258)
		.name(name() + ".avg_ZT")
		.desc("average Number of ZTs of different Hamming distance HD");
	avgST
		.init(258)
		.name(name() + ".avg_ST")
		.desc("average Number of STs of different Hamming distance HD");
	avgHT
		.init(258)
		.name(name() + ".avg_HT")
		.desc("average Number of HTs of different Hamming distance HD");
	avgTT
		.init(258)
		.name(name() + ".avg_TT")
		.desc("average Number of TTs of different Hamming distance HD");
		
		
	avgFlipbits
		.init(9)
		.name(name() + ".avg_fb")
		.desc("average Number of fbs of different Hamming distance HD");	
		
	totalFlipbits
		.init(9)
		.name(name() + ".total_fb")
		.desc("toal Number of fbs of different Hamming distance HD")
		.flags(total | nonan);
		
		
	totalZT
		.init(258)
		.name(name() + ".total_ZT")
		.desc("toal Number of ZT of different Hamming distance HD")
		.flags(total | nonan);
	totalST
		.init(258)
		.name(name() + ".total_ST")
		.desc("toal Number of ST of different Hamming distance HD")
		.flags(total | nonan);
	totalHT
		.init(258)
		.name(name() + ".total_HT")
		.desc("toal Number of HT of different Hamming distance HD")
		.flags(total | nonan);
	totalTT
		.init(258)
		.name(name() + ".total_TT")
		.desc("toal Number of TT of different Hamming distance HD")
		.flags(total | nonan);
		
	lruTrans
		.init(5)
		.name(name() + ".lru_trans")
		.desc("total Number of 4 transitions if lru")
		.flags(total | nonan);
	optimalTrans
		.init(5)
		.name(name() + ".optimal_trans")
		.desc("total Number of 4 transitions if optimamized")
		.flags(total | nonan);


    avgOccs
        .name(name() + ".occ_percent")
        .desc("Average percentage of cache occupancy")
        .flags(nozero | total)
        ;
    for (int i = 0; i < cache->system->maxMasters(); i++) {
        avgOccs.subname(i, cache->system->getMasterName(i));
    }

    avgOccs = occupancies / Stats::constant(numBlocks);

    occupanciesTaskId
        .init(ContextSwitchTaskId::NumTaskId)
        .name(name() + ".occ_task_id_blocks")
        .desc("Occupied blocks per task id")
        .flags(nozero | nonan)
        ;

    ageTaskId
        .init(ContextSwitchTaskId::NumTaskId, 5)
        .name(name() + ".age_task_id_blocks")
        .desc("Occupied blocks per task id")
        .flags(nozero | nonan)
        ;

    percentOccsTaskId
        .name(name() + ".occ_task_id_percent")
        .desc("Percentage of cache occupancy per task id")
        .flags(nozero)
        ;

    percentOccsTaskId = occupanciesTaskId / Stats::constant(numBlocks);

    tagAccesses
        .name(name() + ".tag_accesses")
        .desc("Number of tag accesses")
        ;

    dataAccesses
        .name(name() + ".data_accesses")
        .desc("Number of data accesses")
        ;

    registerDumpCallback(new BaseTagsDumpCallback(this));
    registerExitCallback(new BaseTagsCallback(this));
}
