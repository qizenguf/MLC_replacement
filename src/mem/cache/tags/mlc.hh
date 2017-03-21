/*
 * Copyright (c) 2012-2013 ARM Limited
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
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
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
 */

/**
 * @file
 * Declaration of a LRU tag store.
 * The LRU tags guarantee that the true least-recently-used way in
 * a set will always be evicted.
 */

#ifndef __MEM_CACHE_TAGS_MLC_HH__
#define __MEM_CACHE_TAGS_MLC_HH__

#include "mem/cache/tags/base_set_assoc.hh"
#include "params/MLC.hh"
#include <vector>
#include <unordered_map>

class MLC : public BaseSetAssoc
{
  private:
	  int encodingSize;
	  int shiftSize;
	  int flipSize;
	  int thres;
	  int loc_weight;
	  double diverse_weight;
  public:
    /** Convenience typedef. */
    typedef MLCParams Params;
    typedef unsigned char Byte;
    typedef uint32_t UInt32;

    /**
     * Construct and initialize this tag store.
     */
    MLC(const Params *p);

    /**
     * Destructor
     */
    ~MLC() {}
	static int lineCompare( const Byte* ablock, const Byte* bblock, int size, int shiftSize, int flipSize);
	//std::vector<int> lineCompare_2bit( const Byte* ablock, const Byte* bblock, int size, int shiftSize, int flipSize, int flipBits);
	std::vector<int> lineCompare_2bit_mapping( const Byte* ablock, const Byte* bblock, int size, int shiftSize, int flipSize, int flipBits);
	static std::vector<int> lineCompare_2bit( const Byte* ablock, const Byte* bblock, int size, int shiftSize, int flipSize, int enc);
	static int encodingCompare(const Byte* ablock, const Byte* bblock, int size, int shiftSize, int flipSize, int thres, int encodingSize);
	int encodingCompare_2bit(const Byte* ablock, const Byte* bblock, int size, int shiftSize, int flipSize, int thres, int encodingSize, int zeroWeight = 0);
    CacheBlk* accessBlock(Addr addr, bool is_secure, Cycles &lat,
                         int context_src);
    CacheBlk* findVictim(Addr addr, PacketPtr pkt = nullptr);
	CacheBlk* findVictimPLRU(Addr addr, PacketPtr pkt = nullptr);
    void insertBlock(PacketPtr pkt, BlkType *blk);
    void invalidate(CacheBlk *blk);
};

#endif // __MEM_CACHE_TAGS_MLC_HH__
