#include "hashing_gpu_def.h"
#include "hashingSDFUtils.cl"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Streaming from GPU to CPU: copies only selected blocks/hashEntries to the CPU if outside of the frustum //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


//-------------------------------------------------------
// Pass 1: Find all SDFBlocks that have to be transfered
//-------------------------------------------------------

__kernel void integrateFromGlobalHashPass1Kernel(
			__global struct NuiCLHashEntry*		d_hash,
			__global uint*						d_heap,
			__global uint*						d_heapCounter,
			__global int*						d_hashBucketMutex,
			const uint							hashNumBuckets,
			const uint							hashMaxCollisionLinkedListSize,
			const float							virtualVoxelSize,
			const uint							start,
			const float							radius,
			const float3						cameraPosition,
			__global uint*						d_SDFBlockCounter,
			__global struct NuiCLHashEntry*		d_SDFBlockDescOutput
        )
{
    const uint bucketID = get_global_id(0) + start;
	const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	if(bucketID < hashNumBuckets * HASH_BUCKET_SIZE)
	{
		//HashEntry entry = getHashEntry(g_Hash, bucketID);
		struct NuiCLHashEntry entry = d_hash[bucketID];
		const int3 sdfBlock;
		sdfBlock.x = entry.pos[0];
		sdfBlock.y = entry.pos[1];
		sdfBlock.z = entry.pos[2];
		float3 posWorld = SDFBlockToWorld(sdfBlock, virtualVoxelSize);
		float d = length(posWorld - cameraPosition);

		if (entry.ptr != FREE_ENTRY && d > radius) {
		
			// Write

#ifndef _HANDLE_COLLISIONS
				uint addr = atomic_add(&d_SDFBlockCounter[0], 1);
				d_SDFBlockDescOutput[addr] = entry;
				appendHeap(entry.ptr/linBlockSize, d_heap, d_heapCounter);
				deleteHashEntry(bucketID, d_hash);
#endif
#ifdef _HANDLE_COLLISIONS
				//if there is an offset or hash doesn't belong to the bucket (linked list)
				if (entry.offset != 0 || computeHashPos(sdfBlock, hashNumBuckets) != bucketID / HASH_BUCKET_SIZE) {
					
					if (deleteHashEntryElement(sdfBlock, d_hash, d_heap, d_heapCounter, d_hashBucketMutex, hashNumBuckets, hashMaxCollisionLinkedListSize)) {
						appendHeap(entry.ptr/linBlockSize, d_heap, d_heapCounter);
						uint addr = atomicAdd(&d_SDFBlockCounter[0], 1);
						d_SDFBlockDescOutput[addr] = entry;
					}
				} else {
					uint addr = atomicAdd(&d_SDFBlockCounter[0], 1);
					d_SDFBlockDescOutput[addr] = entry;
					appendHeap(entry.ptr/linBlockSize, d_heap, d_heapCounter);
					deleteHashEntry(bucketID, d_hash);
				}
#endif
		}
	}
}

//-------------------------------------------------------
// Pass 2: Copy SDFBlocks to output buffer
//-------------------------------------------------------
__kernel void integrateFromGlobalHashPass2Kernel(
			__global struct NuiCLVoxel*			d_SDFBlocks,
			__global uint*						d_SDFBlockCounter,
			__global struct NuiCLVoxel*			d_SDFBlockOutput,
			__global struct NuiCLHashEntry*		d_SDFBlockDescOutput
        )
{
	const uint idxBlock = get_group_id(0);

	const uint nSDFBlocks = vload(0, d_SDFBlockCounter);
	if(idxBlock < nSDFBlocks)
	{
		const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
		const uint idxInBlock = get_local_id(0);
		struct NuiCLHashEntry desc = d_SDFBlockDescOutput[idxBlock];

		// Copy SDF block to CPU
		d_SDFBlockOutput[idxBlock*linBlockSize + idxInBlock] = d_SDFBlocks[desc.ptr + idxInBlock];

		//// Reset SDF Block
		deleteSDFVoxel(desc.ptr + idxInBlock, d_SDFBlocks);
	}
}

///////////////////////////////////////////////////////////////////////
// Streaming from CPU to GPU: copies an entire chunk back to the GPU //
///////////////////////////////////////////////////////////////////////



//-------------------------------------------------------
// Pass 1: Allocate memory
//-------------------------------------------------------
__kernel void chunkToGlobalHashPass1Kernel(
			__global uint*						d_heap,
			__global uint*						d_heapCounter,
			__global struct NuiCLHashEntry*		d_hash,
			const uint							hashNumBuckets,
			const uint							hashMaxCollisionLinkedListSize,
			__global struct NuiCLHashEntry*		d_SDFBlockDescInput
        )
{
	const uint bucketID = get_global_id(0);
	const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	const uint heapCountPrev = vload(0, d_heapCounter);
	uint ptr = d_heap[heapCountPrev - bucketID] * linBlockSize;	//mass alloc

	struct NuiCLHashEntry entry = d_SDFBlockDescInput[bucketID];

	//TODO MATTHIAS check this: if this is false, we have a memory leak... -> we need to make sure that this works! (also the next kernel will randomly fill memory)
	bool ok = insertHashEntry(entry, d_hash, hashNumBuckets, hashMaxCollisionLinkedListSize);
}

//-------------------------------------------------------
// Pass 2: Copy input to SDFBlocks
//-------------------------------------------------------

__kernel void chunkToGlobalHashPass2Kernel(
			__global uint*						d_heap,
			__global uint*						d_heapCounter,
			__global struct NuiCLVoxel*			d_SDFBlocks,
			__global struct NuiCLVoxel*			d_SDFBlockInput
			)
{
	const uint blockID = get_group_id(0);
	const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
	const uint heapCountPrev = vload(0, d_heapCounter);
	const uint idxInBlock = get_local_id(0);
	const uint blockSize = get_local_size(0);

	uint ptr = d_heap[heapCountPrev-blockID]*linBlockSize;
	d_SDFBlocks[ptr + idxInBlock] = d_SDFBlockInput[blockID*blockSize + idxInBlock];
	//hashData.d_SDFBlocks[ptr + threadIdx.x].color = make_uchar3(255,0,0);

	//Update heap counter
	uint initialCountNew = heapCountPrev-get_global_size(0);
	vstore(initialCountNew, 0, d_heapCounter);
}
