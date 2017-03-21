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
 * Definitions of a CAR tag store.
 */

#include "mem/cache/tags/car.hh"

#include "debug/CacheRepl.hh"
#include "mem/cache/base.hh"

CAR::CAR(const Params *p)
    : BaseSetAssoc(p)
{
	/*encodingSize = p->encodingSize;
	shiftSize = p->shiftSize;
	flipSize = p->flipSize;
	thres =p->thres;*/
	encodingSize = 8;
	shiftSize = 64;
	flipSize = 8;
	thres = 47;
	loc_weight = 8;
	std::cout<<"car"<<std::endl;
}

CacheBlk*
CAR::accessBlock(Addr addr, bool is_secure, Cycles &lat, int master_id)
{
    CacheBlk *blk = BaseSetAssoc::accessBlock(addr, is_secure, lat, master_id);

    if (blk != nullptr) {
        // move this block to the front of the tree
        sets[blk->set].moveToFront(blk);
        DPRINTF(CacheRepl, "set %x: moving blk %x (%s) to front\n",
                blk->set, regenerateBlkAddr(blk->tag, blk->set),
                is_secure ? "s" : "ns");
    }

    return blk;
}
int 
CAR::lineCompare( const Byte* ablock, const Byte* bblock, int size, int shiftSize, int flipSize){
	if( flipSize == 0 ) flipSize = size+1; // no flip
	int minimal_bits = 512;
	for (int i =0; i<size; i += shiftSize){
		Byte temp;
		int total_bits = 0;
		int flipbits = 0;
		for(int j = 0; j < size; j++){
			temp = ablock[j] ^ bblock[(j+i)%size];
			for(int k = 0; k<8; k++)  {
				if (temp & (1<<k) ) flipbits++; // count every bit in a byte
			}
			if((j+1)% flipSize == 0 ){ // to decide if flip
				if(flipbits > flipSize*4) flipbits = flipSize*8 - flipbits;
				total_bits += flipbits;
				flipbits = 0;	
			}		
		}
		total_bits += flipbits ; 
		minimal_bits = std::min(minimal_bits, total_bits);		
	}
	return minimal_bits;	
}
std::vector<int>
CAR::lineCompare_2bit( const Byte* ablock, const Byte* bblock, int size, int shiftSize, int flipSize, int encodingBits){
	if( flipSize == 0 ) flipSize = size+1; // no flip
	std::unordered_map<int, int> normal_cnt;
	std::unordered_map<int, int> rev_cnt;
	std::vector<int> ret(5, 512);
	//int minimal_bits = 512;
	for (int i =0; i<size; i += shiftSize){
		Byte from, to;
		std::vector<int> res(5,0);// ZT, ST, HT, TT
		//int total_bits = 0;
		//int flipbits = 0;
		bool ifFlip = encodingBits & 1;
		int cnt01 = 0;
		int cnt10 = 0;
		for(int j = 0; j < size; j++){
			from = ablock[j]; // from block
			if(ifFlip) from = ~ from; 
			to = bblock[(j+i)%size]; // to block
			
			
			for(int k = 0; k<8; k += 2)  {
				int label = (((from >> k) & 3)*10) + ((to >> k) & 3); // 3 for 0b11
				int rev_label = (((from >> k) & 3)*10) + ~((to >> k) & 3);
				if(label  == 30 or label == 31 or label == 20 or label == 21) cnt10++; // significant bits change from 1 to 0
				else if(label == 3 or label == 13 or label == 2 or label == 12 ) cnt01 ++;
				normal_cnt[label]++;
				rev_cnt[rev_label]++;// 3 is the mask 0x11. count every 2 bits in a byte
				 
			}
			if((j+1)% flipSize == 0 ){ // to decide if flip
				if(cnt01 + cnt10 <= flipSize*2){
					res[4] = res[4]<<1 ;
					for(auto it : normal_cnt){
						if(it.first == 0 or it.first == 33 or it.first == 22 or it.first == 11 )//zt
							res[0] += it.second;							
						else if(it.first == 1 or it.first == 10 or it.first == 32 or it.first == 23)//st
							res[1] += it.second;
						else if(it.first == 3 or it.first == 13 or it.first == 20 or it.first == 30) //ht
							res[2] += it.second;
						else 
							res[3] += it.second;
					}
				}else{
					res[4] = (res[4]<<1) + 1;
					for(auto it : rev_cnt){
						if(it.first == 0 or it.first == 33 or it.first == 22 or it.first == 11 )//zt
							res[0] += it.second;							
						else if(it.first == 1 or it.first == 10 or it.first == 32 or it.first == 23)//st
							res[1] += it.second;
						else if(it.first == 3 or it.first == 13 or it.first == 20 or it.first == 30) //ht
							res[2] += it.second;
						else 
							res[3] += it.second;
					}
				}
				ifFlip = encodingBits & (1 << ((j+1)/flipSize));
				normal_cnt.clear();
				rev_cnt.clear();
				cnt01 = 0;
				cnt10 = 0;					
			}		
		}
		if(flipSize > size){
				for(auto it : normal_cnt){
						if(it.first == 0 or it.first == 33 or it.first == 22 or it.first == 11 )//zt
							res[0] += it.second;							
						else if(it.first == 1 or it.first == 10 or it.first == 32 or it.first == 23)//st
							res[1] += it.second;
						else if(it.first == 3 or it.first == 13 or it.first == 20 or it.first == 30) //ht
							res[2] += it.second;
						else 
							res[3] += it.second;
				}
		}
		if(res[3] + res[2] < ret[3] + ret[2])
				ret = res;
	}
	assert(ret[3] + ret[2] + ret[0] + ret[1] == 256);
	return ret;	
}

int
CAR::encodingCompare(const Byte* ablock, const Byte* bblock, int size, int shiftSize, int flipSize, int thres, int encodingSize){
	if(flipSize == 0 ) flipSize = size+1;
	int minimal_diff = size/8;
	for (int i =0; i<size; i += shiftSize) {
		//Byte temp;
		int total_diff = 0;
		int abits = 0, bbits = 0;
		for(int j = 0; j<size; j++){
				for(int k = 0; k<8; k++)  {
					if (ablock[j] & (1<<k) ) abits++; // count every bit in a byte
					if (bblock[j] & (1<<k) ) bbits++;
					}
				if( (j+1)%encodingSize == 0 ) {
					if(abits > thres or abits < (encodingSize*8 - thres) ) abits = 1;
					else abits = 0;
					if(bbits > thres or bbits < (encodingSize*8 - thres) ) bbits = 1;
					else bbits = 0;
					if(abits != bbits) total_diff++;
					abits = 0;
					bbits = 0;
				}					
		}
		//total_bits += flipbits;  // if no flip..
		if(total_diff < minimal_diff) minimal_diff = total_diff;		
	}
	//assert(minimal_diff <= size/encodingSize);
	return minimal_diff;	
}

CacheBlk*
CAR::findVictim(Addr addr, PacketPtr pkt)
{
    //int set = extractSet(addr);
    // grab a replacement candidate
    //CacheBlk *blk = BaseSetAssoc::findVictim(addr);
    //bool unfilled = true ;
    
    //LRU framework 
	int set = extractSet(addr);
    // grab a replacement candidate
    BlkType *blk = nullptr;
    BlkType *ret = nullptr;
    int cur_min = INT_MAX;
    std::vector<int> lru_trans;
    std::vector<int> opt_trans;
    bool lru = true;
    int rind = assoc - 1;
    for (int i = assoc - 1; i >= 0; i--) { // replace the second to LRU
        BlkType *b = sets[set].blks[i];
        if (b->way < allocAssoc) {
            blk = b;           
			lru_trans = lineCompare_2bit(blk->data, pkt->getConstPtr<uint8_t>(), 64, shiftSize, flipSize, sets[set].flipBits[blk->way]);				
			if(lru){
				for(int i =0; i < 4; i++)
						lruTrans[i] += lru_trans[i];
				lru = false;
				if(i != assoc -1){
					opt_trans = lru_trans;
					//cur_min = lru_trans[2]+lru_trans[3];
					rind = i;
					ret = blk;
					break;
				}
					
			}
			if(lru_trans[2]+lru_trans[3] < cur_min){
				opt_trans = lru_trans;
				cur_min = lru_trans[2]+lru_trans[3];
				rind = i;
				ret = blk;
			}			
        }       
    }
    for(int i = 0; i < 4 ; i++)
		optimalTrans[i] += opt_trans[i];
		
	sets[set].flipBits[ret->way] = 	opt_trans[4];
    rind = rind;
    assert(!ret || ret->way < allocAssoc);
	
    if (ret && ret->isValid()) {
        DPRINTF(CacheRepl, "set %x: selecting blk %x for replacement at %d\n", set, regenerateBlkAddr(ret->tag, set), rind);
    }
    return ret;
    
    std::vector<int> locVal(assoc, 0);
    //LRU ENDS
    int retValue = -1;
    
	//replacement process
	if ( blk && blk->isValid()) {
		//int retValue = -1;
		if(range == 8){
			int plru[16] = {0};
			plru[0] = sets[set].m_tree[0];
			plru[1] = sets[set].m_tree[1];
			plru[2] = sets[set].m_tree[16];
			plru[3] = sets[set].m_tree[2];
			plru[4] = sets[set].m_tree[9];
			plru[5] = sets[set].m_tree[17];
			plru[6] = sets[set].m_tree[24];
			for(int i = 7; i<15; i++){
				int temp = 0;
				int p = i;
				int level = 0;
				while( p != 0){
					if(p%2==0 && plru[p/2-1] ==1) temp = temp+ (1<<level);
					else if( p%2 == 1 && plru[p/2] == 0) temp = temp + (1<<level);
					level ++;
					p = (p-1)/2;
				}
				if(temp == 7 ) // temp higher, more lru
					plru[i] = 1; // LRU one, the plru value is smallest as 1.
				else if(temp == 6) 
					plru[i] = 2;
				else if(temp > 3)	
					plru[i] = 3;
				else 
					plru[i] = 4;
				}
			
				 if ( sets[set].m_tree[3] == 0)
				 {
					if ( sets[set].m_tree[4] == 0) retValue= 0;
					else           retValue= 1;  // b2==1
				 }
				 else
				 {                            // b1==1
					if ( sets[set].m_tree[5] == 0) retValue = 2;
					else           retValue = 3;  // b3==1
				 }
				 locVal[retValue]=plru[7];                         // b0==1
				 if ( sets[set].m_tree[6] == 0)
				 {
					if ( sets[set].m_tree[7] == 0) retValue = 4;
					else           retValue = 5;  // b5==1
				 }
				 else
				 {                            // b4==1
					if ( sets[set].m_tree[8] == 0) retValue = 6;
					else           retValue = 7;  // b6==1
				}
				locVal[retValue] = (plru[8]);   
			 
				 if ( sets[set].m_tree[10] == 0)
				 {
					if ( sets[set].m_tree[11] == 0) retValue= 8;
					else           retValue= 9;  // b2==1
				 }
				 else
				 {                            // b1==1
					if ( sets[set].m_tree[12] == 0) retValue = 10;
					else           retValue = 11;  // b3==1
				 }
					locVal[retValue] = (plru[9]);   // b0==1
				 if ( sets[set].m_tree[13] == 0)
				 {
					if ( sets[set].m_tree[14] == 0) retValue = 12;
					else           retValue = 13;  // b5==1
				 }
				 else
				 {                            // b4==1
					if ( sets[set].m_tree[15] == 0) retValue = 14;
					else           retValue = 15;  // b6==1
				 }
				locVal[retValue] = (plru[10]);   
			  
				 if ( sets[set].m_tree[18] == 0)
				 {
					if ( sets[set].m_tree[19] == 0) retValue= 16;
					else            retValue= 17;  // b2==1
				 }
				 else
				 {                            // b1==1
					if ( sets[set].m_tree[20] == 0) retValue = 18;
					else           retValue = 19;  // b3==1
				 }
				locVal[retValue] = (plru[11]);                               // b0==1
				 if ( sets[set].m_tree[21] == 0)
				 {
					if ( sets[set].m_tree[22] == 0) retValue = 20;
					else           retValue = 21;  // b5==1
				 }
				 else
				 {                            // b4==1
					if ( sets[set].m_tree[23] == 0) retValue = 22;
					else           retValue = 23;  // b6==1
				 }
				locVal[retValue] = (plru[12]);    
			 
				 if ( sets[set].m_tree[25] == 0)
				 {
					if ( sets[set].m_tree[26] == 0) retValue= 24;
					else           retValue= 25;  // b2==1
				 }
				 else
				 {                            // b1==1
					if ( sets[set].m_tree[27] == 0) retValue = 26;
					else           retValue = 27;  // b3==1
				 }
				locVal[retValue] = (plru[13]);    
										   // b0==1
				 if ( sets[set].m_tree[28] == 0)
				 {
					if ( sets[set].m_tree[29] == 0) retValue = 28;
					else           retValue = 29;  // b5==1
				 }
				 else
				 {                            // b4==1
					if ( sets[set].m_tree[30] == 0) retValue = 30;
					else           retValue = 31;  // b6==1
				 }	
				locVal[retValue] = (plru[14]);  
				} 
				if(range == 4){
					int plru[8] = {0};
					plru[0] = sets[set].m_tree[0];
					plru[1] = sets[set].m_tree[1];
					plru[2] = sets[set].m_tree[16];

					for(int i = 3; i<7; i++){
						int temp = 0;
						int p = i;
						int level = 0;
						while( p != 0){
							if(p%2==0 && plru[p/2-1] ==1) temp = temp+ (1<<level);
							else if( p%2 == 1 && plru[p/2] == 0) temp = temp + (1<<level);
							level ++;
							p = (p-1)/2;
						}
						if(temp == 3 )
							plru[i] = 1;
						else if(temp == 2) 
							plru[i] = 2;
						else if(temp ==1)	
							plru[i] = 2;
						else
							plru[i] = 2;
						}
			
				  
					  if ( sets[set].m_tree[2] == 0)
					  {
						 if ( sets[set].m_tree[3] == 0)
						 {
							if ( sets[set].m_tree[4] == 0) retValue= 0;
							else           retValue= 1;  // b2==1
						 }
						 else
						 {                            // b1==1
							if ( sets[set].m_tree[5] == 0) retValue = 2;
							else           retValue = 3;  // b3==1
						 }
					  }
					  else
					  {                               // b0==1
						 if ( sets[set].m_tree[6] == 0)
						 {
							if ( sets[set].m_tree[7] == 0) retValue = 4;
							else           retValue = 5;  // b5==1
						 }
						 else
						 {                            // b4==1
							if ( sets[set].m_tree[8] == 0) retValue = 6;
							else           retValue = 7;  // b6==1
						 }
					  }
						locVal[retValue] = (plru[3]);  
					
					
					  if ( sets[set].m_tree[9] == 0)
					  {
						 if ( sets[set].m_tree[10] == 0)
						 {
							if ( sets[set].m_tree[11] == 0) retValue= 8;
							else           retValue= 9;  // b2==1
						 }
						 else
						 {                            // b1==1
							if ( sets[set].m_tree[12] == 0) retValue = 10;
							else           retValue = 11;  // b3==1
						 }
					  }
					  else
					  {                               // b0==1
						 if ( sets[set].m_tree[13] == 0)
						 {
							if ( sets[set].m_tree[14] == 0) retValue = 12;
							else           retValue = 13;  // b5==1
						 }
						 else
						 {                            // b4==1
							if ( sets[set].m_tree[15] == 0) retValue = 14;
							else           retValue = 15;  // b6==1
						 }
					  }
						locVal[retValue] = (plru[4]);  
					
					  if ( sets[set].m_tree[17] == 0)
					  {
						 if ( sets[set].m_tree[18] == 0)
						 {
							if ( sets[set].m_tree[19] == 0) retValue= 16;
							else            retValue= 17;  // b2==1
						 }
						 else
						 {                            // b1==1
							if ( sets[set].m_tree[20] == 0) retValue = 18;
							else           retValue = 19;  // b3==1
						 }
					  }
					  else
					  {                               // b0==1
						 if ( sets[set].m_tree[21] == 0)
						 {
							if ( sets[set].m_tree[22] == 0) retValue = 20;
							else           retValue = 21;  // b5==1
						 }
						 else
						 {                            // b4==1
							if ( sets[set].m_tree[23] == 0) retValue = 22;
							else           retValue = 23;  // b6==1
						 }
					  }
					
						locVal[retValue] = (plru[5]);  
					  if ( sets[set].m_tree[24] == 0)
					  {
						 if ( sets[set].m_tree[25] == 0)
						 {
							if ( sets[set].m_tree[26] == 0) retValue= 24;
							else           retValue= 25;  // b2==1
						 }
						 else
						 {                            // b1==1
							if ( sets[set].m_tree[27] == 0) retValue = 26;
							else           retValue = 27;  // b3==1
						 }
					  }
					  else
					  {                               // b0==1
						 if ( sets[set].m_tree[28] == 0)
						 {
							if ( sets[set].m_tree[29] == 0) retValue = 28;
							else           retValue = 29;  // b5==1
						 }
						 else
						 {                            // b4==1
							if ( sets[set].m_tree[30] == 0) retValue = 30;
							else           retValue = 31;  // b6==1
						 }
					  }
					locVal[retValue] = (plru[6]);  
					
				}
				int min_index = 0;
				 int hd = 0;
				 int cur_hd = 0;
				 int cur_recency = 0;
				 int lowestDiffBits =INT_MAX;
				 
				 for(UInt32 i = 0 ; i < assoc; i++)
				{	
					int recency = locVal[i];
					if(recency != 0 ){
						//if( range == 0 && recency == 1) recency = 0;
						//set->read_line(i, 0, read_buff, 64, false);
						//std::memcpy(blk->data, pkt->getConstPtr<uint8_t>(), blkSize);
						hd = encodingCompare(sets[set].blks[i]->data, pkt->getConstPtr<uint8_t>(), 64, shiftSize, flipSize, thres, encodingSize);
						int t = (8 * encodingSize) * hd  + recency * loc_weight;
						//std::cout<< i <<" diff "<<t<<" bits "<<(uint8_t)read_buff[0]<<std::endl;
						if(t < lowestDiffBits || (t == lowestDiffBits && recency < cur_recency)) { 
						//std::cout<< i <<" diff "<<t<<std::endl;
							lowestDiffBits = t; 
							min_index = i;
							cur_hd = hd;
							cur_recency = recency;
						}
					}
				}
				int idx = min_index;
				blk = sets[set].blks[idx];
				
				while (blk->way >= allocAssoc) {
					idx = (idx + 1) % assoc;
					blk = sets[set].blks[idx];
				}

				assert(idx < assoc);
				assert(idx >= 0);
				assert(blk->way < allocAssoc);
				
		//assert(!blk || blk->way < allocAssoc);
				int fb = lineCompare(blk->data, pkt->getConstPtr<uint8_t>(), 64, shiftSize, flipSize);
				totalFlipbits[cur_hd] += fb;
				avgFlipbits[cur_hd] = fb;
				DPRINTF(CacheRepl, "set %x: selecting blk %x for plru replacement with hd = %d %d\n", set, regenerateBlkAddr(blk->tag, set), cur_hd, fb);	
			}
			
			DPRINTF(CacheRepl, "set %x: selecting blk %x for plru replacement with \n", set, regenerateBlkAddr(blk->tag, set));
				
		return blk;
}



CacheBlk*
CAR::findVictimPLRU(Addr addr, PacketPtr pkt)
{
    int set = extractSet(addr);
    // grab a replacement candidate
    BlkType *blk = nullptr;
    
    for (int i = assoc - 1; i >= 0; i--) {
        BlkType *b = sets[set].blks[i];
        if (b->way < allocAssoc) {
            blk = b;
            break;
        }
    }
    assert(!blk || blk->way < allocAssoc);
    int retValue = -1;
    
    if (blk && blk->isValid()) {//replacement process
	
	
	   if( assoc == 1 ) retValue = 0;
	   if (assoc == 4)
	   {
		  if (sets[set].m_tree[0] == 0)
		  {
			 if (sets[set].m_tree[1] == 0) retValue = 0;
			 else           retValue = 1;   // b1==1
		  }
		  else
		  {
			 if (sets[set].m_tree[2] == 0) retValue = 2;
			 else           retValue = 3;   // b2==1
		  }
	   }
	   else if (assoc == 8)
	   {
		  if (sets[set].m_tree[0] == 0)
		  {  
			 if (sets[set].m_tree[1] == 0)
			 {
				if (sets[set].m_tree[2] == 0) retValue= 0;
				else           retValue= 1;  // b2==1
			 }
			 else
			 {                            // b1==1
				if (sets[set].m_tree[3] == 0) retValue = 2;
				else           retValue = 3;  // b3==1
			 }
		  }
		  else
		  {                               // b0==1
			 if (sets[set].m_tree[4] == 0)
			 {
				if (sets[set].m_tree[5] == 0) retValue = 4;
				else           retValue = 5;  // b5==1
			 }
			 else
			 {                            // b4==1
				if (sets[set].m_tree[6] == 0) retValue = 6;
				else           retValue = 7;  // b6==1
			 }
		  }
	   }
	   else if (assoc == 16)
	   {
		  if (sets[set].m_tree[0] == 0)
			  {
				  if (sets[set].m_tree[1] == 0)
				  {
					 if (sets[set].m_tree[2] == 0)
					 {
						if (sets[set].m_tree[3] == 0) retValue= 0;
						else           retValue= 1;  // b2==1
					 }
					 else
					 {                            // b1==1
						if (sets[set].m_tree[4] == 0) retValue = 2;
						else           retValue = 3;  // b3==1
					 }
				  }
				  else
				  {                               // b0==1
					 if (sets[set].m_tree[5] == 0)
					 {
						if (sets[set].m_tree[6] == 0) retValue = 4;
						else           retValue = 5;  // b5==1
					 }
					 else
					 {                            // b4==1
						if (sets[set].m_tree[7] == 0) retValue = 6;
						else           retValue = 7;  // b6==1
					 }
				  }
				}
				else{
				  if (sets[set].m_tree[8] == 0)
				  {
					 if (sets[set].m_tree[9] == 0)
					 {
						if (sets[set].m_tree[10] == 0) retValue= 8;
						else           retValue= 9;  // b2==1
					 }
					 else
					 {                            // b1==1
						if (sets[set].m_tree[11] == 0) retValue = 10;
						else           retValue = 11;  // b3==1
					 }
				  }
				  else
				  {                               // b0==1
					 if (sets[set].m_tree[12] == 0)
					 {
						if (sets[set].m_tree[13] == 0) retValue = 12;
						else           retValue = 13;  // b5==1
					 }
					 else
					 {                            // b4==1
						if (sets[set].m_tree[14] == 0) retValue = 14;
						else           retValue = 15;  // b6==1
					 }
				  }
				}
			}
		else if (assoc == 32)
		{
			  if( sets[set].m_tree[0] == 0 ){
				  
				  if( sets[set].m_tree[1] == 0 )
				  {
					  if (sets[set].m_tree[2] == 0)
					  {
						 if (sets[set].m_tree[3] == 0)
						 {
							if (sets[set].m_tree[4] == 0) retValue= 0;
							else           retValue= 1;  // b2==1
						 }
						 else
						 {                            // b1==1
							if (sets[set].m_tree[5] == 0) retValue = 2;
							else           retValue = 3;  // b3==1
						 }
					  }
					  else
					  {                               // b0==1
						 if (sets[set].m_tree[6] == 0)
						 {
							if (sets[set].m_tree[7] == 0) retValue = 4;
							else           retValue = 5;  // b5==1
						 }
						 else
						 {                            // b4==1
							if (sets[set].m_tree[8] == 0) retValue = 6;
							else           retValue = 7;  // b6==1
						 }
					  }
					}
					else{
					  if (sets[set].m_tree[9] == 0)
					  {
						 if (sets[set].m_tree[10] == 0)
						 {
							if (sets[set].m_tree[11] == 0) retValue= 8;
							else           retValue= 9;  // b2==1
						 }
						 else
						 {                            // b1==1
							if (sets[set].m_tree[12] == 0) retValue = 10;
							else           retValue = 11;  // b3==1
						 }
					  }
					  else
					  {                               // b0==1
						 if (sets[set].m_tree[13] == 0)
						 {
							if (sets[set].m_tree[14] == 0) retValue = 12;
							else           retValue = 13;  // b5==1
						 }
						 else
						 {                            // b4==1
							if (sets[set].m_tree[15] == 0) retValue = 14;
							else           retValue = 15;  // b6==1
						 }
					  }
					}
				}
				else{
					if( sets[set].m_tree[16] == 0 )
					{
					  if (sets[set].m_tree[17] == 0)
					  {
						 if (sets[set].m_tree[18] == 0)
						 {
							if (sets[set].m_tree[19] == 0) retValue= 16;
							else            retValue= 17;  // b2==1
						 }
						 else
						 {                            // b1==1
							if (sets[set].m_tree[20] == 0) retValue = 18;
							else           retValue = 19;  // b3==1
						 }
					  }
					  else
					  {                               // b0==1
						 if (sets[set].m_tree[21] == 0)
						 {
							if (sets[set].m_tree[22] == 0) retValue = 20;
							else           retValue = 21;  // b5==1
						 }
						 else
						 {                            // b4==1
							if (sets[set].m_tree[23] == 0) retValue = 22;
							else           retValue = 23;  // b6==1
						 }
					  }
					}
					else{
					  if (sets[set].m_tree[24] == 0)
					  {
						 if (sets[set].m_tree[25] == 0)
						 {
							if (sets[set].m_tree[26] == 0) retValue= 24;
							else           retValue= 25;  // b2==1
						 }
						 else
						 {                            // b1==1
							if (sets[set].m_tree[27] == 0) retValue = 26;
							else           retValue = 27;  // b3==1
						 }
					  }
					  else
					  {                               // b0==1
						 if (sets[set].m_tree[28] == 0)
						 {
							if (sets[set].m_tree[29] == 0) retValue = 28;
							else           retValue = 29;  // b5==1
						 }
						 else
						 {                            // b4==1
							if (sets[set].m_tree[30] == 0) retValue = 30;
							else           retValue = 31;  // b6==1
						 }
					  }
					}
				}
			}
		
	 blk = sets[set].blks[retValue];	
     DPRINTF(CacheRepl, "set %x: selecting blk %x for plru replacement\n",
                set, regenerateBlkAddr(blk->tag, set));
    }
    
    return blk;
}

void
CAR::insertBlock(PacketPtr pkt, BlkType *blk)
{
    BaseSetAssoc::insertBlock(pkt, blk);

    int set = extractSet(pkt->getAddr());
    sets[set].moveToFront(blk);
}

void
CAR::invalidate(CacheBlk *blk)
{
    BaseSetAssoc::invalidate(blk);

    // should be evicted before valid blocks
   // int set = blk->set;
    //sets[set].moveToTail(blk);
}

CAR*
CARParams::create()
{
    return new CAR(this);
}
