#pragma once


//status flags for hash entries
#define LOCK_ENTRY -1
#define FREE_ENTRY -2
#define NO_OFFSET 0

#define SDF_BLOCK_SIZE 8
#define HASH_BUCKET_SIZE 8

#define SDF_BLOCK_SIZE3 512				// SDF_BLOCK_SIZE3 = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE
#define SDF_LOCAL_BLOCK_NUM 0x100000		// Number of locally stored blocks, currently 2^17

#define SDF_GLOBAL_BLOCK_NUM 0x240000	// Number of globally stored blocks: SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE
#define SDF_TRANSFER_BLOCK_NUM 0x1000	// Maximum number of blocks transfered in one swap operation

#define SDF_BUCKET_NUM 0x150000			// Number of Hash Bucket, should be 2^n and bigger than SDF_LOCAL_BLOCK_NUM, SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define SDF_HASH_MASK 0x14ffff			// Used for get hashing value of the bucket index,  SDF_HASH_MASK = SDF_BUCKET_NUM - 1
#define SDF_EXCESS_LIST_SIZE 0x30000	// 0x20000 Size of excess list, used to handle collisions. Also max offset (unsigned short) value.


#define FP_MINF -FLT_MAX
#define FP_PINF FLT_MAX

#pragma pack(16)
struct NuiCLHashEntry
{
	int				pos[3];		//hash position (lower left corner of SDFBlock))
	int				ptr;		//pointer into heap to SDFBlock
	unsigned int	offset;		//offset for collisions
};
#pragma pack()

#pragma pack(8)
struct NuiCLVoxel
{
	float			sdf;		//signed distance function
	unsigned char	color[3];	//color 
	unsigned char	weight;		//accumulated sdf weight
};
#pragma pack()


#define FAR_AWAY 999999.9f
#define VERY_CLOSE 0.05f


#pragma pack(16)
struct NuiKinfuHashEntry
{
	/** Position of the corner of the 8x8x8 volume, that identifies the entry. */
	short			pos[3];		//hash position (lower left corner of SDFBlock))
	/** Pointer to the voxel block array.
	    - >= 0 identifies an actual allocated entry in the voxel block array
	    - -1 identifies an entry that has been removed (swapped out)
	    - <-1 identifies an unallocated block
	*/
	int				ptr;		//pointer into heap to SDFBlock
	/** Offset in the excess list. */
	unsigned int	offset;		//offset for collisions
};
#pragma pack()

#pragma pack(8)
struct NuiKinfuVoxel
{
	short			sdf;		//signed distance function
	unsigned char	color[3];	//color
	unsigned char	weight;		//accumulated sdf weight
};
#pragma pack()

/// 0 - most recent data is on host, data not currently in active
///     memory
/// 1 - data both on host and in active memory, information has not
///     yet been combined
/// 2 - most recent data is in active memory, should save this data
///     back to host at some point
typedef unsigned char NuiKinfuHashSwapState;
