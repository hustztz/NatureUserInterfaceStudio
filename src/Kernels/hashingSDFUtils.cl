#include "cameraUtils.cl"
#include "hashing_gpu_def.h"

#define _HANDLE_COLLISIONS

inline static uint computeHashPos(int3 virtualVoxelPos, const uint hashNumBuckets)
{ 
	const int p0 = 73856093;
	const int p1 = 19349669;
	const int p2 = 83492791;

	int res = ((virtualVoxelPos.x * p0) ^ (virtualVoxelPos.y * p1) ^ (virtualVoxelPos.z * p2)) % hashNumBuckets;
	if (res < 0) res += hashNumBuckets;
	return convert_uint(res);
}

inline static uint consumeHeap( __global uint*	d_heap,
								__global uint*	d_heapCounter)
{
	uint addr = atomic_dec(&d_heapCounter[0]);
	//TODO MATTHIAS check some error handling?
	return vload(addr, d_heap);
}

inline static void appendHeap(uint ptr,
							  __global uint*	d_heap,
							  __global uint*	d_heapCounter)
{
	uint addr = atomic_inc(&(d_heapCounter[0]));
	//TODO MATTHIAS check some error handling?
	d_heap[addr+1] = ptr;
}

inline static int3 virtualVoxelPosToSDFBlock(int3 virtualVoxelPos)
{
	if (virtualVoxelPos.x < 0) virtualVoxelPos.x -= SDF_BLOCK_SIZE-1;
	if (virtualVoxelPos.y < 0) virtualVoxelPos.y -= SDF_BLOCK_SIZE-1;
	if (virtualVoxelPos.z < 0) virtualVoxelPos.z -= SDF_BLOCK_SIZE-1;

	return (int3)(
		virtualVoxelPos.x/SDF_BLOCK_SIZE,
		virtualVoxelPos.y/SDF_BLOCK_SIZE,
		virtualVoxelPos.z/SDF_BLOCK_SIZE);
}

inline static int3 worldToVirtualVoxelPos(float3 pos, const float virtualVoxelSize)
{
	//const float3 p = pos*g_VirtualVoxelResolutionScalar;
	const float3 p = pos / virtualVoxelSize;
	return convert_int3(p + convert_float3(sign(p))*0.5f);
}

inline static int3 SDFBlockToVirtualVoxelPos(int3 sdfBlock)
{
	return sdfBlock*SDF_BLOCK_SIZE;
}

inline static float3 virtualVoxelPosToWorld(int3 pos, const float virtualVoxelSize)
{
	return convert_float3(pos)*virtualVoxelSize;
}

inline static int3 worldToSDFBlock(float3 worldPos, float virtualVoxelSize)
{
	return virtualVoxelPosToSDFBlock(worldToVirtualVoxelPos(worldPos, virtualVoxelSize));
}

inline static float3 SDFBlockToWorld(int3 sdfBlock, const float virtualVoxelSize)
{
	return virtualVoxelPosToWorld(SDFBlockToVirtualVoxelPos(sdfBlock), virtualVoxelSize);
}

//! computes the linearized index of a local virtual voxel pos; pos in [0;7]^3
inline static int3 delinearizeVoxelIndex(uint idx)
{
	int x = idx % SDF_BLOCK_SIZE;
	int y = (idx % (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE;
	int z = idx / (SDF_BLOCK_SIZE * SDF_BLOCK_SIZE);
	return (int3)(x,y,z);
}

//! returns the hash entry for a given sdf block id; if there was no hash entry the returned entry will have a ptr with FREE_ENTRY set
inline static struct NuiCLHashEntry getHashEntryForSDFBlockPos(
	int3			sdfBlock,
	__global struct NuiCLHashEntry*	d_hash,
	const uint		hashNumBuckets,
	const uint		hashMaxCollisionLinkedListSize
	)
{
	uint h = computeHashPos(sdfBlock, hashNumBuckets);			//hash bucket
	uint hp = h * HASH_BUCKET_SIZE;	//hash position

	struct NuiCLHashEntry entry;
	entry.pos[0] = sdfBlock.x;
	entry.pos[1] = sdfBlock.y;
	entry.pos[2] = sdfBlock.z;
	entry.offset = 0;
	entry.ptr = FREE_ENTRY;

	for (uint j = 0; j < HASH_BUCKET_SIZE; j++) {
		uint i = j + hp;
		struct NuiCLHashEntry curr = d_hash[i];
		if (curr.pos[0] == entry.pos[0] && curr.pos[1] == entry.pos[1] && curr.pos[2] == entry.pos[2] && curr.ptr != FREE_ENTRY) {
			return curr;
		}
	}

#ifdef _HANDLE_COLLISIONS
	const uint idxLastEntryInBucket = (h+1)*HASH_BUCKET_SIZE - 1;
	int i = idxLastEntryInBucket;	//start with the last entry of the current bucket
	struct NuiCLHashEntry curr;
	//traverse list until end: memorize idx at list end and memorize offset from last element of bucket to list end

	unsigned int maxIter = 0;
	uint g_MaxLoopIterCount = hashMaxCollisionLinkedListSize;
	while (maxIter < g_MaxLoopIterCount) {
		curr = d_hash[i];

		if (curr.pos[0] == entry.pos[0] && curr.pos[1] == entry.pos[1] && curr.pos[2] == entry.pos[2] && curr.ptr != FREE_ENTRY) {
			return curr;
		}

		if (curr.offset == 0) {	//we have found the end of the list
			break;
		}
		i = idxLastEntryInBucket + curr.offset;						//go to next element in the list
		i %= (HASH_BUCKET_SIZE * hashNumBuckets);	//check for overflow

		maxIter++;
	}
#endif
	return entry;
}

//! returns the hash entry for a given worldPos; if there was no hash entry the returned entry will have a ptr with FREE_ENTRY set
inline static struct NuiCLHashEntry getHashEntry(
	float3 worldPos,
	__global struct NuiCLHashEntry*	d_hash,
	const float		virtualVoxelSize,
	const uint		hashNumBuckets,
	const uint		hashMaxCollisionLinkedListSize)
{
	//int3 blockID = worldToSDFVirtualVoxelPos(worldPos)/SDF_BLOCK_SIZE;	//position of sdf block
	int3 blockID = worldToSDFBlock(worldPos, virtualVoxelSize);
	return getHashEntryForSDFBlockPos(blockID, d_hash, hashNumBuckets, hashMaxCollisionLinkedListSize);
}


inline static void deleteHashEntry(uint idx, __global struct NuiCLHashEntry*	d_hash)
{
	d_hash[idx].ptr = FREE_ENTRY;
	d_hash[idx].offset = 0;
	d_hash[idx].pos[0] = d_hash[idx].pos[1] = d_hash[idx].pos[2] = 0;
}

inline static void allocBlock(int3			pos,
						 __global struct NuiCLHashEntry*	d_hash,
						 __global uint*	d_heap,
						 __global uint*	d_heapCounter,
						 __global int*	d_hashBucketMutex,
						 const uint		hashNumBuckets,
						 const uint		hashMaxCollisionLinkedListSize)
{
	uint h = computeHashPos(pos, hashNumBuckets);				//hash bucket
	uint hp = h * HASH_BUCKET_SIZE;	//hash position

	int firstEmpty = -1;
	for (uint j = 0; j < HASH_BUCKET_SIZE; j++) {
		uint i = hp + j;		
		struct NuiCLHashEntry curr = d_hash[i];

		//in that case the SDF-block is already allocated and corresponds to the current position -> exit thread
		if (curr.pos[0] == pos.x && curr.pos[1] == pos.y && curr.pos[2] == pos.z && curr.ptr != FREE_ENTRY) {
			return;
		}

		//store the first FREE_ENTRY hash entry
		if (firstEmpty == -1 && curr.ptr == FREE_ENTRY) {
			firstEmpty = i;
		}
	}


#ifdef _HANDLE_COLLISIONS
	//updated variables as after the loop
	const uint idxLastEntryInBucket = (h+1)*HASH_BUCKET_SIZE - 1;	//get last index of bucket
	uint i = idxLastEntryInBucket;											//start with the last entry of the current bucket
	//int offset = 0;
	struct NuiCLHashEntry curr;
	curr.offset = 0;
	//traverse list until end: memorize idx at list end and memorize offset from last element of bucket to list end
	//int k = 0;

	unsigned int maxIter = 0;
	while (maxIter < hashMaxCollisionLinkedListSize)
	{
		//offset = curr.offset;
		curr = d_hash[i];	//TODO MATTHIAS do by reference
		if (curr.pos[0] == pos.x && curr.pos[1] == pos.y && curr.pos[2] == pos.z && curr.ptr != FREE_ENTRY) {
			return;
		}
		if (curr.offset == 0) {	//we have found the end of the list
			break;
		}
		i = idxLastEntryInBucket + curr.offset;		//go to next element in the list
		i %= (HASH_BUCKET_SIZE * hashNumBuckets);	//check for overflow

		maxIter++;
	}
#endif

	if (firstEmpty != -1) {	//if there is an empty entry and we haven't allocated the current entry before
		//int prevValue = 0;
		//InterlockedExchange(d_hashBucketMutex[h], LOCK_ENTRY, prevValue);	//lock the hash bucket
		int prevValue = atomic_xchg(&(d_hashBucketMutex[h]), LOCK_ENTRY);
		if (prevValue != LOCK_ENTRY) {	//only proceed if the bucket has been locked
			struct NuiCLHashEntry entry;
			entry.pos[0] = pos.x;
			entry.pos[1] = pos.y;
			entry.pos[2] = pos.z;
			entry.offset = NO_OFFSET;		
			entry.ptr = consumeHeap(d_heap, d_heapCounter) * SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;	//memory alloc
			d_hash[firstEmpty] = entry;
		}
		return;
	}

#ifdef _HANDLE_COLLISIONS
	//if (i != idxLastEntryInBucket) return;
	int offset = 0;
	//linear search for free entry

	maxIter = 0; 
	while (maxIter < hashMaxCollisionLinkedListSize) {
		offset++;
		i = (idxLastEntryInBucket + offset) % (HASH_BUCKET_SIZE * hashNumBuckets);	//go to next hash element
		if ((offset % HASH_BUCKET_SIZE) == 0) continue;			//cannot insert into a last bucket element (would conflict with other linked lists)
		curr = d_hash[i];
		//if (curr.pos.x == pos.x && curr.pos.y == pos.y && curr.pos.z == pos.z && curr.ptr != FREE_ENTRY) {
		//	return;
		//} 
		if (curr.ptr == FREE_ENTRY) {	//this is the first free entry
			//int prevValue = 0;
			//InterlockedExchange(g_HashBucketMutex[h], LOCK_ENTRY, prevValue);	//lock the original hash bucket
			int prevValue = atomic_xchg(&(d_hashBucketMutex[h]), LOCK_ENTRY);
			if (prevValue != LOCK_ENTRY) {
				struct NuiCLHashEntry lastEntryInBucket = d_hash[idxLastEntryInBucket];
				h = i / HASH_BUCKET_SIZE;
				//InterlockedExchange(g_HashBucketMutex[h], LOCK_ENTRY, prevValue);	//lock the hash bucket where we have found a free entry
				prevValue = atomic_xchg(&d_hashBucketMutex[h], LOCK_ENTRY);
				if (prevValue != LOCK_ENTRY) {	//only proceed if the bucket has been locked
					struct NuiCLHashEntry entry;
					entry.pos[0] = pos.x;
					entry.pos[1] = pos.y;
					entry.pos[2] = pos.z;
					entry.offset = lastEntryInBucket.offset;		
					entry.ptr = consumeHeap(d_heap, d_heapCounter) * SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;	//memory alloc
					d_hash[i] = entry;

					lastEntryInBucket.offset = offset;
					d_hash[idxLastEntryInBucket] = lastEntryInBucket;
					//setHashEntry(g_Hash, idxLastEntryInBucket, lastEntryInBucket);
				}
			} 
			return;	//bucket was already locked
		}

		maxIter++;
	}
#endif
}

//!inserts a hash entry without allocating any memory: used by streaming: TODO MATTHIAS check the atomics in this function
inline static bool insertHashEntry(
							struct NuiCLHashEntry			entry,
							__global struct NuiCLHashEntry*	d_hash,
							const uint						hashNumBuckets,
							const uint						hashMaxCollisionLinkedListSize
							)
{
	const int3 sdfBlock;
	sdfBlock.x = entry.pos[0];
	sdfBlock.y = entry.pos[1];
	sdfBlock.z = entry.pos[2];
	uint h = computeHashPos(sdfBlock, hashNumBuckets);
	uint hp = h * HASH_BUCKET_SIZE;

	for (uint j = 0; j < HASH_BUCKET_SIZE; j++) {
		uint i = j + hp;		
		//const HashEntry& curr = d_hash[i];
		int prevWeight = 0;
		//InterlockedCompareExchange(hash[3*i+2], FREE_ENTRY, LOCK_ENTRY, prevWeight);
		prevWeight = atom_cmpxchg(&d_hash[i].ptr, FREE_ENTRY, LOCK_ENTRY);
		if (prevWeight == FREE_ENTRY) {
			d_hash[i] = entry;
			//setHashEntry(hash, i, entry);
			return true;
		}
	}

#ifdef _HANDLE_COLLISIONS
	//updated variables as after the loop
	const uint idxLastEntryInBucket = (h+1)*HASH_BUCKET_SIZE - 1;	//get last index of bucket

	uint i = idxLastEntryInBucket;											//start with the last entry of the current bucket
	struct NuiCLHashEntry curr;

	unsigned int maxIter = 0;
	//[allow_uav_condition]
	uint g_MaxLoopIterCount = hashMaxCollisionLinkedListSize;
	while (maxIter < g_MaxLoopIterCount) {									//traverse list until end // why find the end? we you are inserting at the start !!!
		//curr = getHashEntry(hash, i);
		curr = d_hash[i];	//TODO MATTHIAS do by reference
		if (curr.offset == 0) break;									//we have found the end of the list
		i = idxLastEntryInBucket + curr.offset;							//go to next element in the list
		i %= (HASH_BUCKET_SIZE * hashNumBuckets);	//check for overflow

		maxIter++;
	}

	maxIter = 0;
	int offset = 0;
	while (maxIter < g_MaxLoopIterCount) {													//linear search for free entry
		offset++;
		uint i = (idxLastEntryInBucket + offset) % (HASH_BUCKET_SIZE * hashNumBuckets);	//go to next hash element
		if ((offset % HASH_BUCKET_SIZE) == 0) continue;										//cannot insert into a last bucket element (would conflict with other linked lists)

		//InterlockedCompareExchange(hash[3*i+2], FREE_ENTRY, LOCK_ENTRY, prevWeight);		//check for a free entry
		int prevWeight = atom_cmpxchg(&d_hash[idxLastEntryInBucket].ptr, (uint)FREE_ENTRY, (uint)LOCK_ENTRY);
		if (prevWeight == FREE_ENTRY) {														//if free entry found set prev->next = curr & curr->next = prev->next
			//[allow_uav_condition]
			//while(hash[3*idxLastEntryInBucket+2] == LOCK_ENTRY); // expects setHashEntry to set the ptr last, required because pos.z is packed into the same value -> prev->next = curr -> might corrput pos.z

			struct NuiCLHashEntry lastEntryInBucket = d_hash[idxLastEntryInBucket];			//get prev (= lastEntry in Bucket)

			int newOffsetPrev = (offset << 16) | (lastEntryInBucket.pos[2] & 0x0000ffff);	//prev->next = curr (maintain old z-pos)
			//InterlockedExchange(hash[3*idxLastEntryInBucket+1], newOffsetPrev, oldOffsetPrev);	//set prev offset atomically
			int oldOffsetPrev = prevWeight = atomic_xchg(&d_hash[idxLastEntryInBucket].ptr, newOffsetPrev);
			entry.offset = oldOffsetPrev >> 16;													//remove prev z-pos from old offset

			//setHashEntry(hash, i, entry);														//sets the current hashEntry with: curr->next = prev->next
			d_hash[i] = entry;
			return true;
		}

		maxIter++;
	} 
#endif

	return false;
}

//! deletes a hash entry position for a given sdfBlock index (returns true uppon successful deletion; otherwise returns false)
inline static bool deleteHashEntryElement(
	int3			sdfBlock,
	__global struct NuiCLHashEntry*	d_hash,
	__global uint*	d_heap,
	__global uint*	d_heapCounter,
	__global int*	d_hashBucketMutex,
	const uint		hashNumBuckets,
	const uint		hashMaxCollisionLinkedListSize
	)
{
	uint h = computeHashPos(sdfBlock, hashNumBuckets);	//hash bucket
	uint hp = h * HASH_BUCKET_SIZE;		//hash position

	for (uint j = 0; j < HASH_BUCKET_SIZE; j++) {
		uint i = j + hp;
		struct NuiCLHashEntry curr = d_hash[i];
		if (curr.pos[0] == sdfBlock.x && curr.pos[1] == sdfBlock.y && curr.pos[2] == sdfBlock.z && curr.ptr != FREE_ENTRY) {
#ifndef _HANDLE_COLLISIONS
			const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
			appendHeap(curr.ptr / linBlockSize, d_heap, d_heapCounter);
			//heapAppend.Append(curr.ptr / linBlockSize);
			deleteHashEntry(i, d_hash);
			return true;
#endif
#ifdef _HANDLE_COLLISIONS
			if (curr.offset != 0) {	//if there was a pointer set it to the next list element
				//int prevValue = 0;
				//InterlockedExchange(bucketMutex[h], LOCK_ENTRY, prevValue);	//lock the hash bucket
				int prevValue = atomic_xchg(&(d_hashBucketMutex[h]), LOCK_ENTRY);
				if (prevValue == LOCK_ENTRY)	return false;
				if (prevValue != LOCK_ENTRY) {
					const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
					appendHeap(curr.ptr / linBlockSize, d_heap, d_heapCounter);
					//heapAppend.Append(curr.ptr / linBlockSize);
					int nextIdx = (i + curr.offset) % (HASH_BUCKET_SIZE*hashNumBuckets);
					//setHashEntry(hash, i, getHashEntry(hash, nextIdx));
					curr = d_hash[nextIdx];
					deleteHashEntry(nextIdx, d_hash);
					return true;
				}
			} else {
				const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
				appendHeap(curr.ptr / linBlockSize, d_heap, d_heapCounter);
				//heapAppend.Append(curr.ptr / linBlockSize);
				deleteHashEntry(i, d_hash);
				return true;
			}
#endif	//HANDLE_COLLSISION
		}
	}	
#ifdef _HANDLE_COLLISIONS
	const uint idxLastEntryInBucket = (h+1)*HASH_BUCKET_SIZE - 1;
	int i = idxLastEntryInBucket;
	struct NuiCLHashEntry curr = d_hash[i];
	int prevIdx = i;
	i = idxLastEntryInBucket + curr.offset;							//go to next element in the list
	i %= (HASH_BUCKET_SIZE * hashNumBuckets);	//check for overflow

	unsigned int maxIter = 0;
	uint g_MaxLoopIterCount = hashMaxCollisionLinkedListSize;

	while (maxIter < g_MaxLoopIterCount) {
		curr = d_hash[i];
		//found that dude that we need/want to delete
		if (curr.pos[0] == sdfBlock.x && curr.pos[1] == sdfBlock.y && curr.pos[2] == sdfBlock.z && curr.ptr != FREE_ENTRY) {
			//int prevValue = 0;
			//InterlockedExchange(bucketMutex[h], LOCK_ENTRY, prevValue);	//lock the hash bucket
			int prevValue = atomic_xchg(&(d_hashBucketMutex[h]), LOCK_ENTRY);
			if (prevValue == LOCK_ENTRY)	return false;
			if (prevValue != LOCK_ENTRY) {
				const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
				appendHeap(curr.ptr / linBlockSize, d_heap, d_heapCounter);
				//heapAppend.Append(curr.ptr / linBlockSize);
				deleteHashEntry(i, d_hash);
				struct NuiCLHashEntry prev = d_hash[prevIdx];				
				prev.offset = curr.offset;
				//setHashEntry(hash, prevIdx, prev);
				d_hash[prevIdx] = prev;
				return true;
			}
		}

		if (curr.offset == 0) {	//we have found the end of the list
			return false;	//should actually never happen because we need to find that guy before
		}
		prevIdx = i;
		i = idxLastEntryInBucket + curr.offset;		//go to next element in the list
		i %= (HASH_BUCKET_SIZE * hashNumBuckets);	//check for overflow

		maxIter++;
	}
#endif	// HANDLE_COLLSISION
	return false;
}


//! computes the linearized index of a local virtual voxel pos; pos in [0;7]^3
inline static uint linearizeVoxelPos(int3 virtualVoxelPos)
{
	return  
		virtualVoxelPos.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE +
		virtualVoxelPos.y * SDF_BLOCK_SIZE +
		virtualVoxelPos.x;
}

inline static int virtualVoxelPosToLocalSDFBlockIndex(int3 virtualVoxelPos)
{
	int3 localVoxelPos = (int3)(
		virtualVoxelPos.x % SDF_BLOCK_SIZE,
		virtualVoxelPos.y % SDF_BLOCK_SIZE,
		virtualVoxelPos.z % SDF_BLOCK_SIZE);

	if (localVoxelPos.x < 0) localVoxelPos.x += SDF_BLOCK_SIZE;
	if (localVoxelPos.y < 0) localVoxelPos.y += SDF_BLOCK_SIZE;
	if (localVoxelPos.z < 0) localVoxelPos.z += SDF_BLOCK_SIZE;

	return linearizeVoxelPos(localVoxelPos);
}

inline static void deleteVoxel(struct NuiCLVoxel v)
{
	v.weight = 0;
	v.sdf = 0.0f;
	v.color[0] = v.color[1] = v.color[2] = 0;
}

inline static void deleteSDFVoxel(uint idx, __global struct NuiCLVoxel*	d_SDFBlocks)
{
	deleteVoxel(d_SDFBlocks[idx]);
}

inline static struct NuiCLVoxel getVoxel(
	float3 worldPos,
	__global struct NuiCLHashEntry*	d_hash,
	__global struct NuiCLVoxel*		d_SDFBlocks,
	const float						virtualVoxelSize,
	const uint						hashNumBuckets,
	const uint						hashMaxCollisionLinkedListSize)
{
	struct NuiCLHashEntry hashEntry = getHashEntry(worldPos, d_hash, virtualVoxelSize, hashNumBuckets, hashMaxCollisionLinkedListSize);
	struct NuiCLVoxel v;
	if (hashEntry.ptr == FREE_ENTRY) {
		deleteVoxel(v);			
	} else {
		int3 virtualVoxelPos = worldToVirtualVoxelPos(worldPos, virtualVoxelSize);
		v = d_SDFBlocks[hashEntry.ptr + virtualVoxelPosToLocalSDFBlockIndex(virtualVoxelPos)];
	}
	return v;
}

inline static struct NuiCLVoxel getSDFVoxel(
	int3 virtualVoxelPos,
	__global struct NuiCLHashEntry*	d_hash,
	__global struct NuiCLVoxel*		d_SDFBlocks,
	const uint						hashNumBuckets,
	const uint						hashMaxCollisionLinkedListSize)
{
	struct NuiCLHashEntry hashEntry = getHashEntryForSDFBlockPos(virtualVoxelPosToSDFBlock(virtualVoxelPos), d_hash, hashNumBuckets, hashMaxCollisionLinkedListSize);
	struct NuiCLVoxel v;
	if (hashEntry.ptr == FREE_ENTRY) {
		deleteVoxel(v);			
	} else {
		v = d_SDFBlocks[hashEntry.ptr + virtualVoxelPosToLocalSDFBlockIndex(virtualVoxelPos)];
	}
	return v;
}
