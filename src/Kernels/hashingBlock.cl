#include "hashingUtils.cl"

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

__kernel void allocBlock(int3			pos,
						 __global uint*	d_hash,
						 __global uint*	d_heap,
						__global uint*	d_heapCounter
						 const uint		hashNumBuckets,
						 const uint		hashMaxCollisionLinkedListSize)
{
	uint h = computeHashPos(pos, hashNumBuckets);				//hash bucket
	uint hp = h * HASH_BUCKET_SIZE;	//hash position

	int firstEmpty = -1;
	for (uint j = 0; j < HASH_BUCKET_SIZE; j++) {
		uint i = j + hp;		
		struct NuiHashEntry curr = vload(i, d_hash);

		//in that case the SDF-block is already allocated and corresponds to the current position -> exit thread
		if (curr.pos.x == pos.x && curr.pos.y == pos.y && curr.pos.z == pos.z && curr.ptr != FREE_ENTRY) {
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
		curr = vload(i, d_hash);	//TODO MATTHIAS do by reference
		if (curr.pos.x == pos.x && curr.pos.y == pos.y && curr.pos.z == pos.z && curr.ptr != FREE_ENTRY) {
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
		int prevValue = atomic_xchg(&d_hashBucketMutex[h], LOCK_ENTRY);
		if (prevValue != LOCK_ENTRY) {	//only proceed if the bucket has been locked
			struct NuiHashEntry entry;
			entry.pos = pos;
			entry.offset = NO_OFFSET;		
			entry.ptr = consumeHeap(d_heap, d_heapCounter) * SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;	//memory alloc
			vstore(entry, firstEmpty, d_hash);
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
		curr = vload(i, d_hash);
		//if (curr.pos.x == pos.x && curr.pos.y == pos.y && curr.pos.z == pos.z && curr.ptr != FREE_ENTRY) {
		//	return;
		//} 
		if (curr.ptr == FREE_ENTRY) {	//this is the first free entry
			//int prevValue = 0;
			//InterlockedExchange(g_HashBucketMutex[h], LOCK_ENTRY, prevValue);	//lock the original hash bucket
			int prevValue = atomic_xchg(&d_hashBucketMutex[h], LOCK_ENTRY);
			if (prevValue != LOCK_ENTRY) {
				struct NuiHashEntry lastEntryInBucket = vload(idxLastEntryInBucket, d_hash);
				h = i / HASH_BUCKET_SIZE;
				//InterlockedExchange(g_HashBucketMutex[h], LOCK_ENTRY, prevValue);	//lock the hash bucket where we have found a free entry
				prevValue = atomic_xchg(&d_hashBucketMutex[h], LOCK_ENTRY);
				if (prevValue != LOCK_ENTRY) {	//only proceed if the bucket has been locked
					struct NuiHashEntry entry;
					entry.pos = pos;
					entry.offset = lastEntryInBucket.offset;		
					entry.ptr = consumeHeap(d_heap, d_heapCounter) * SDF_BLOCK_SIZE*SDF_BLOCK_SIZE*SDF_BLOCK_SIZE;	//memory alloc
					vstore(entry, i, d_hash);

					lastEntryInBucket.offset = offset;
					vstore(lastEntryInBucket, idxLastEntryInBucket, d_hash);
					//setHashEntry(g_Hash, idxLastEntryInBucket, lastEntryInBucket);
				}
			} 
			return;	//bucket was already locked
		}

		maxIter++;
	}
//#endif
}
