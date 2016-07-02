#pragma once

//status flags for hash entries
#define LOCK_ENTRY -1
#define FREE_ENTRY -2
#define NO_OFFSET 0

#define SDF_BLOCK_SIZE 8
#define HASH_BUCKET_SIZE 10

struct NuiCLHashEntry
{
	int				pos[3];		//hash position (lower left corner of SDFBlock))
	int				ptr;		//pointer into heap to SDFBlock
	unsigned int	offset;		//offset for collisions
};

struct NuiCLVoxel
{
	float			sdf;		//signed distance function
	unsigned char	color[3];	//color 
	unsigned char	weight;		//accumulated sdf weight
};