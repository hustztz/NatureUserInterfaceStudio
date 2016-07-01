#include "hashingUtils.cl"
#include "hashing_gpu_def.h"

inline static uint computeHashPos(int3 virtualVoxelPos, uint hashNumBuckets)
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
	uint addr = atomic_sub(&d_heapCounter[0], 1);
	//TODO MATTHIAS check some error handling?
	return vload(addr, d_heap);
}

inline static void appendHeap(uint ptr,
							  __global uint*	d_heap,
							  __global uint*	d_heapCounter)
{
	uint addr = atomic_add(&(d_heapCounter[0]), 1);
	//TODO MATTHIAS check some error handling?
	d_heap[addr+1] = ptr;
}

inline static void allocBlock(int3			pos,
						 __global struct NuiHashEntry*	d_hash,
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
		uint i = j + hp;		
		struct NuiHashEntry curr = d_hash[i];

		//in that case the SDF-block is already allocated and corresponds to the current position -> exit thread
		if (curr.pos[0] == pos.x && curr.pos[1] == pos.y && curr.pos[2] == pos.z && curr.ptr != FREE_ENTRY) {
			return;
		}

		//store the first FREE_ENTRY hash entry
		if (firstEmpty == -1 && curr.ptr == FREE_ENTRY) {
			firstEmpty = i;
		}
	}


//#ifdef _HANDLE_COLLISIONS
	//updated variables as after the loop
	const uint idxLastEntryInBucket = (h+1)*HASH_BUCKET_SIZE - 1;	//get last index of bucket
	uint i = idxLastEntryInBucket;											//start with the last entry of the current bucket
	//int offset = 0;
	struct NuiHashEntry curr;
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
//#endif

	if (firstEmpty != -1) {	//if there is an empty entry and we haven't allocated the current entry before
		//int prevValue = 0;
		//InterlockedExchange(d_hashBucketMutex[h], LOCK_ENTRY, prevValue);	//lock the hash bucket
		int prevValue = atomic_xchg(&(d_hashBucketMutex[h]), LOCK_ENTRY);
		if (prevValue != LOCK_ENTRY) {	//only proceed if the bucket has been locked
			struct NuiHashEntry entry;
			entry.pos[0] = pos.x;
			entry.pos[1] = pos.y;
			entry.pos[2] = pos.z;
			entry.offset = NO_OFFSET;		
			entry.ptr = consumeHeap(d_heap, d_heapCounter) * SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;	//memory alloc
			d_hash[firstEmpty] = entry;
		}
		return;
	}

//#ifdef _HANDLE_COLLISIONS
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
				struct NuiHashEntry lastEntryInBucket = d_hash[idxLastEntryInBucket];
				h = i / HASH_BUCKET_SIZE;
				//InterlockedExchange(g_HashBucketMutex[h], LOCK_ENTRY, prevValue);	//lock the hash bucket where we have found a free entry
				prevValue = atomic_xchg(&d_hashBucketMutex[h], LOCK_ENTRY);
				if (prevValue != LOCK_ENTRY) {	//only proceed if the bucket has been locked
					struct NuiHashEntry entry;
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
//#endif
}

inline static bool deleteHashEntryElement(
	int3			sdfBlock,
	__global struct NuiHashEntry*	d_hash,
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
		struct NuiHashEntry curr = d_hash[i];
		if (curr.pos[0] == sdfBlock.x && curr.pos[1] == sdfBlock.y && curr.pos[2] == sdfBlock.z && curr.ptr != FREE_ENTRY) {
//#ifndef HANDLE_COLLISIONS
			const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
			appendHeap(curr.ptr / linBlockSize, d_heap, d_heapCounter);
			//heapAppend.Append(curr.ptr / linBlockSize);
			deleteHashEntry(i, d_hash);
			return true;
//#endif
//#ifdef HANDLE_COLLISIONS
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
//#endif	//HANDLE_COLLSISION
		}
	}	
//#ifdef HANDLE_COLLISIONS
	const uint idxLastEntryInBucket = (h+1)*HASH_BUCKET_SIZE - 1;
	int i = idxLastEntryInBucket;
	struct NuiHashEntry curr = d_hash[i];
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
			int prevValue = atomic_exch(&(d_hashBucketMutex[h]), LOCK_ENTRY);
			if (prevValue == LOCK_ENTRY)	return false;
			if (prevValue != LOCK_ENTRY) {
				const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
				appendHeap(curr.ptr / linBlockSize, d_heap, d_heapCounter);
				//heapAppend.Append(curr.ptr / linBlockSize);
				deleteHashEntry(i, d_hash);
				struct NuiHashEntry prev = d_hash[prevIdx];				
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
//#endif	// HANDLE_COLLSISION
	return false;
}
