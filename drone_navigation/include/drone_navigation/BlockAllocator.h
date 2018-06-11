/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __BLOCKALLOCATOR_H__
#define __BLOCKALLOCATOR_H__

#include <cstdint>

/// This is a small object allocator used for allocating small
/// objects that persist for more than one time step.
/// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
class BlockAllocator
{
    static const int ChunkSize = 16 * 1024;
    static const int MaxBlockSize = 640;
    static const int BlockSizes = 14;
    static const int ChunkArrayIncrement = 128;

public:
    BlockAllocator();
    ~BlockAllocator();

public:
    void* Allocate(int size);
    void Free(void *p, int size);
    void Clear();

private:
    int             num_chunk_count_;
    int             num_chunk_space_;
    struct Chunk*   chunks_;
    struct Block*   free_lists_[BlockSizes];
    static int      block_sizes_[BlockSizes];
    static uint8_t  s_block_size_lookup_[MaxBlockSize + 1];
    static bool     s_block_size_lookup_initialized_;
};

#endif
