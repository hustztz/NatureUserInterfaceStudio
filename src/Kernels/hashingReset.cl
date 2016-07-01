#include "hashingUtils.cl"

__kernel void reset_heap_kernel(
            __global uint*	d_heap,
			__global uint*	d_heapCounter,
			__global struct NuiVoxel*	d_SDFBlocks
        )
{
    const uint gidx = get_global_id(0);
	const uint gsizex = get_global_size(0);

	if (gidx == 0) {
		vstore(gsizex - 1, 0, d_heapCounter);	//points to the last element of the array
	}
	
	vstore(gsizex - gidx - 1, gidx, d_heap);
	uint blockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
	uint base_idx = gidx * blockSize;
	for (uint i = 0; i < blockSize; i++)
	{
		deleteVoxel(base_idx+i, d_SDFBlocks);
	}
}

__kernel void reset_hash_kernel(
            __global struct NuiHashEntry*	d_hash,
			__global struct NuiHashEntry*	d_hashCompactified
        )
{
    const uint gidx = get_global_id(0);
	deleteHashEntry(gidx, d_hash);
	deleteHashEntry(gidx, d_hashCompactified);
}

__kernel void reset_hash_bucket_mutex_kernel(
            __global char*	d_hashBucketMutex
        )
{
    const uint gidx = get_global_id(0);
	vstore(FREE_ENTRY, gidx, d_hashBucketMutex);
}
