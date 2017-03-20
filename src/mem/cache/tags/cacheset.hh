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
 * Copyright (c) 2009 The Regents of The University of Michigan
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
 * Authors: Lisa Hsu
 */

/**
 * @file
 * Declaration of an associative set
 */

#ifndef __MEM_CACHE_TAGS_CACHESET_HH__
#define __MEM_CACHE_TAGS_CACHESET_HH__

#include <cassert>

/**
 * An associative set of cache blocks.
 */
template <class Blktype>
class CacheSet
{
  public:
    /** The associativity of this set. */
    int assoc;
	
    /** Cache blocks in this set, maintained in LRU order 0 = MRU. */
    Blktype **blks;
	uint8_t* m_tree;
	int* flipBits;
    /**
     * Find a block matching the tag in this set.
     * @param way_id The id of the way that matches the tag.
     * @param tag The Tag to find.
     * @param is_secure True if the target memory space is secure.
     * @return Pointer to the block if found. Set way_id to assoc if none found
     */
    Blktype* findBlk(Addr tag, bool is_secure, int& way_id) const ;
    Blktype* findBlk(Addr tag, bool is_secure) const ;

    /**
     * Move the given block to the head of the list.
     * @param blk The block to move.
     */
    void moveToHead(Blktype *blk);
    void moveToFront(Blktype *blk);

    /**
     * Move the given block to the tail of the list.
     * @param blk The block to move
     */
    void moveToTail(Blktype *blk);

};

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag, bool is_secure, int& way_id) const
{
    /**
     * Way_id returns the id of the way that matches the block
     * If no block is found way_id is set to assoc.
     */
    way_id = assoc;
    for (int i = 0; i < assoc; ++i) {
        if (blks[i]->tag == tag && blks[i]->isValid() &&
            blks[i]->isSecure() == is_secure) {
            way_id = i;
            return blks[i];
        }
    }
    return nullptr;
}

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag, bool is_secure) const
{
    int ignored_way_id;
    return findBlk(tag, is_secure, ignored_way_id);
}

template <class Blktype>
void
CacheSet<Blktype>::moveToHead(Blktype *blk)
{
    // nothing to do if blk is already head
    if (blks[0] == blk)
        return;

    // write 'next' block into blks[i], moving up from MRU toward LRU
    // until we overwrite the block we moved to head.

    // start by setting up to write 'blk' into blks[0]
    int i = 0;
    Blktype *next = blk;

    do {
        assert(i < assoc);
        std::swap(blks[i], next);
        ++i;
    } while (next != blk);
}
template <class Blktype>
void
CacheSet<Blktype>::moveToFront(Blktype *blk) // Pseudo LRU
{

    int i = 0;
	int accessed_index = -1;
    /*for( ; i<assoc; i++ ){
		if(blks[i] == blk) accessed_index = i;
	}*/
	Blktype *next = blk;

    do {
        assert(i < assoc);
        //std::swap(blks[i], next);
        next = blks[i];
        ++i;
    } while (next != blk);
    accessed_index = i-1;
	assert(accessed_index != -1);
    
	if (assoc == 4)
	{
      if      (accessed_index==0) { m_tree[0]=1;m_tree[1]=1;     }
      else if (accessed_index==1) { m_tree[0]=1;m_tree[1]=0;     }
      else if (accessed_index==2) { m_tree[0]=0;       m_tree[2]=1;}
      else if (accessed_index==3) { m_tree[0]=0;       m_tree[2]=0;}
   }
	else if (assoc == 8)
   {
      if      (accessed_index==0) { m_tree[0]=1;m_tree[1]=1;m_tree[2]=1;                            }
      else if (accessed_index==1) { m_tree[0]=1;m_tree[1]=1;m_tree[2]=0;                            }
      else if (accessed_index==2) { m_tree[0]=1;m_tree[1]=0;       m_tree[3]=1;                     }
      else if (accessed_index==3) { m_tree[0]=1;m_tree[1]=0;       m_tree[3]=0;                     }
      else if (accessed_index==4) { m_tree[0]=0;                     m_tree[4]=1;m_tree[5]=1;       }
      else if (accessed_index==5) { m_tree[0]=0;                     m_tree[4]=1;m_tree[5]=0;       }
      else if (accessed_index==6) { m_tree[0]=0;                     m_tree[4]=0;       m_tree[6]=1;}
      else if (accessed_index==7) { m_tree[0]=0;                     m_tree[4]=0;       m_tree[6]=0;}
   }
	else if (assoc == 16) // added by Qi
	{
      if      (accessed_index==0) { m_tree[0]=1;m_tree[1]=1;m_tree[2]=1;m_tree[3]=1;                           }
      else if (accessed_index==1) { m_tree[0]=1;m_tree[1]=1;m_tree[2]=1;m_tree[3]=0;                            }
      else if (accessed_index==2) { m_tree[0]=1;m_tree[1]=1;m_tree[2]=0;       m_tree[4]=1;                     }
      else if (accessed_index==3) { m_tree[0]=1;m_tree[1]=1;m_tree[2]=0;       m_tree[4]=0;                     }
      else if (accessed_index==4) { m_tree[0]=1;m_tree[1]=1;                     m_tree[5]=1;m_tree[6]=1;       }
      else if (accessed_index==5) { m_tree[0]=1;m_tree[1]=1;                     m_tree[5]=1;m_tree[6]=0;       }
      else if (accessed_index==6) { m_tree[0]=1;m_tree[1]=1;                     m_tree[5]=0;       m_tree[7]=1;}
      else if (accessed_index==7) { m_tree[0]=1;m_tree[1]=0;                     m_tree[5]=0;       m_tree[7]=0;}
      else if (accessed_index==8) { m_tree[0]=0; m_tree[8]=1;m_tree[9]=1;m_tree[10]=1;                            }
      else if (accessed_index==9) { m_tree[0]=0; m_tree[8]=1;m_tree[9]=1;m_tree[10]=0;                            }
      else if (accessed_index==10) { m_tree[0]=0; m_tree[8]=1;m_tree[9]=0;       m_tree[11]=1;                     }
      else if (accessed_index==11) { m_tree[0]=0; m_tree[8]=1;m_tree[9]=0;       m_tree[11]=0;                     }
      else if (accessed_index==12) { m_tree[0]=0; m_tree[8]=0;                     m_tree[12]=1;m_tree[13]=1;       }
      else if (accessed_index==13) { m_tree[0]=0; m_tree[8]=0;                     m_tree[12]=1;m_tree[13]=0;       }
      else if (accessed_index==14) { m_tree[0]=0; m_tree[8]=0;                     m_tree[12]=0;       m_tree[14]=1;}
      else if (accessed_index==15) { m_tree[0]=0; m_tree[8]=0;                     m_tree[12]=0;       m_tree[14]=0;}
   }
	else if (assoc == 32) // added by Qi
   {
      if      (accessed_index==0) { m_tree[0]=1; m_tree[1]=1;m_tree[2]=1;m_tree[3]=1;m_tree[4]=1;                           } // 0000 00001
      else if (accessed_index==1) { m_tree[0]=1; m_tree[1]=1;m_tree[2]=1;m_tree[3]=1;m_tree[4]=0;                            }
      else if (accessed_index==2) { m_tree[0]=1; m_tree[1]=1;m_tree[2]=1;m_tree[3]=0;	      m_tree[5]=1;                     }
      else if (accessed_index==3) { m_tree[0]=1; m_tree[1]=1;m_tree[2]=1;m_tree[3]=0;         m_tree[5]=0;                     }
      else if (accessed_index==4) { m_tree[0]=1; m_tree[1]=1;m_tree[2]=1;                     m_tree[6]=1;m_tree[7]=1;       }
      else if (accessed_index==5) { m_tree[0]=1; m_tree[1]=1;m_tree[2]=1;                     m_tree[6]=1;m_tree[7]=0;       }
      else if (accessed_index==6) { m_tree[0]=1; m_tree[1]=1;m_tree[2]=1;                     m_tree[6]=0;       m_tree[8]=1;}
      else if (accessed_index==7) { m_tree[0]=1; m_tree[1]=1;m_tree[2]=0;                     m_tree[6]=0;       m_tree[8]=0;}
      else if (accessed_index==8) { m_tree[0]=1; m_tree[1]=0; m_tree[9]=1;m_tree[10]=1;m_tree[11]=1;                            }
      else if (accessed_index==9) { m_tree[0]=1; m_tree[1]=0; m_tree[9]=1;m_tree[10]=1;m_tree[11]=0;                            }
      else if (accessed_index==10) { m_tree[0]=1; m_tree[1]=0; m_tree[9]=1;m_tree[10]=0;        m_tree[12]=1;                     }
      else if (accessed_index==11) { m_tree[0]=1; m_tree[1]=0; m_tree[9]=1;m_tree[10]=0;        m_tree[12]=0;                     }
      else if (accessed_index==12) { m_tree[0]=1; m_tree[1]=0; m_tree[9]=0;                     m_tree[13]=1;m_tree[14]=1;       }
      else if (accessed_index==13) { m_tree[0]=1; m_tree[1]=0; m_tree[9]=0;                     m_tree[13]=1;m_tree[14]=0;       }
      else if (accessed_index==14) { m_tree[0]=1; m_tree[1]=0; m_tree[9]=0;                     m_tree[13]=0;       m_tree[15]=1;}
      else if (accessed_index==15) { m_tree[0]=1; m_tree[1]=0; m_tree[9]=0;                     m_tree[13]=0;       m_tree[15]=0;}
      
      else if (accessed_index== 16) { m_tree[0]=0; m_tree[16]=1;m_tree[17]=1;m_tree[18]=1;m_tree[19]=1;                           } // 0000 00001
      else if (accessed_index== 17) { m_tree[0]=0; m_tree[16]=1;m_tree[17]=1;m_tree[18]=1;m_tree[19]=0;                            }
      else if (accessed_index== 18) { m_tree[0]=0; m_tree[16]=1;m_tree[17]=1;m_tree[18]=0;       m_tree[20]=1;                     }
      else if (accessed_index== 19) { m_tree[0]=0; m_tree[16]=1;m_tree[17]=1;m_tree[18]=0;       m_tree[20]=0;                     }
      else if (accessed_index== 20) { m_tree[0]=0; m_tree[16]=1;m_tree[17]=1;                    m_tree[21]=1;m_tree[22]=1;       }
      else if (accessed_index==21) { m_tree[0]=0; m_tree[16]=1;m_tree[17]=1;                     m_tree[21]=1;m_tree[22]=0;       }
      else if (accessed_index==22) { m_tree[0]=0; m_tree[16]=1;m_tree[17]=1;                     m_tree[21]=0;       m_tree[23]=1;}
      else if (accessed_index==23) { m_tree[0]=0; m_tree[16]=1;m_tree[17]=0;                     m_tree[21]=0;       m_tree[23]=0;}
      else if (accessed_index==24) { m_tree[0]=0; m_tree[16]=0; m_tree[24]=1;m_tree[25]=1;m_tree[26]=1;                            }
      else if (accessed_index==25) { m_tree[0]=0; m_tree[16]=0; m_tree[24]=1;m_tree[25]=1;m_tree[26]=0;                            }
      else if (accessed_index==26) { m_tree[0]=0; m_tree[16]=0; m_tree[24]=1;m_tree[25]=0;        m_tree[27]=1;                     }
      else if (accessed_index==27) { m_tree[0]=0; m_tree[16]=0; m_tree[24]=1;m_tree[25]=0;        m_tree[27]=0;                     }
      else if (accessed_index==28) { m_tree[0]=0; m_tree[16]=0; m_tree[24]=0;                     m_tree[28]=1;m_tree[29]=1;       }
      else if (accessed_index==29) { m_tree[0]=0; m_tree[16]=0; m_tree[24]=0;                     m_tree[28]=1;m_tree[29]=0;       }
      else if (accessed_index==30) { m_tree[0]=0; m_tree[16]=0; m_tree[24]=0;                     m_tree[28]=0;       m_tree[30]=1;}
      else if (accessed_index==31) { m_tree[0]=0; m_tree[16]=0; m_tree[24]=0;                     m_tree[28]=0;       m_tree[30]=0;}
   }
   else
   { 
	   return;
     // LOG_PRINT_ERROR("PLRU doesn't support associativity %d", assoc);
   }
   return;
    
    
}

template <class Blktype>
void
CacheSet<Blktype>::moveToTail(Blktype *blk)
{
    // nothing to do if blk is already tail
    if (blks[assoc - 1] == blk)
        return;

    // write 'next' block into blks[i], moving from LRU to MRU
    // until we overwrite the block we moved to tail.

    // start by setting up to write 'blk' into tail
    int i = assoc - 1;
    Blktype *next = blk;

    do {
        assert(i >= 0);
        std::swap(blks[i], next);
        --i;
    } while (next != blk);
    
}

#endif
