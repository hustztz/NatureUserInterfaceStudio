#include "KinfuHashUtils.cl"
#include "prefixSum.cl"
#include "utils.cl"

__kernel void fetchHashScene_kernel(
            __global float*					d_vmap,
			__global float*					d_cmap,
			__global volatile int*				vertex_id,
			__global	int*					d_visibleEntryIDs,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global struct NuiKinfuVoxel*		d_SDFBlocks,
			const		float					virtualVoxelSize,
			const		float					truncScale)
{
	const uint gidx = get_global_id(0);
	const int entryID = d_visibleEntryIDs[gidx];
	if(entryID < 0)
		return;
	const struct NuiKinfuHashEntry hashEntry = d_hashEntry[entryID];
	if (hashEntry.ptr < 0)
		return;

	for (int i = 0; i < SDF_BLOCK_SIZE3; i++)
	{
		const int blockId = mul24(hashEntry.ptr, SDF_BLOCK_SIZE3) + i;
		const struct NuiKinfuVoxel localVoxelBlock = d_SDFBlocks[blockId];
		if ( localVoxelBlock.weight > 0 && abs(localVoxelBlock.sdf) < 32767 * truncScale ){ //mu=0.02
			float3 pt;
			pt.z = (hashEntry.pos[2] * SDF_BLOCK_SIZE + (i / (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE) + 0.5f)) *virtualVoxelSize;
			pt.y = (hashEntry.pos[1] * SDF_BLOCK_SIZE + ((i % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) / SDF_BLOCK_SIZE + 0.5f)) *virtualVoxelSize;
			pt.x = (hashEntry.pos[0] * SDF_BLOCK_SIZE + ((i % (SDF_BLOCK_SIZE*SDF_BLOCK_SIZE)) % SDF_BLOCK_SIZE + 0.5f)) *virtualVoxelSize;

			float4 color_value = (float4)(0.0f, 0.0f, 0.0f, 1.0f);
			if(d_cmap)
			{
				color_value.x = convert_float(localVoxelBlock.color[0]) /255.f;
				color_value.y = convert_float(localVoxelBlock.color[1]) /255.f;
				color_value.z = convert_float(localVoxelBlock.color[2]) /255.f;
				color_value.w = 1.0f;
			}

			int current_id = atomic_inc(vertex_id);
			if(current_id < MAX_OUTPUT_VERTEX_SIZE)
			{
				vstore3(pt, current_id, d_vmap);
				vstore4(color_value, current_id, d_cmap);
				//d_vmap[current_id] = pt;
				//d_cmap[current_id] = color_value;
			}
		}
	}
}

inline static int findVoxelBlockId(const int3 virtualPos,
							__global struct NuiKinfuHashEntry*	d_hashEntry
							)
{
	int3 blockPos = virtualPosToVoxelBlock(virtualPos);
	int linearIdx = virtualPos.x + mul24((virtualPos.y - blockPos.x), SDF_BLOCK_SIZE) + mul24(mul24((virtualPos.z - blockPos.y), SDF_BLOCK_SIZE), SDF_BLOCK_SIZE) - mul24(blockPos.z, SDF_BLOCK_SIZE3);
	short3 shortBlockPos = convert_short3(blockPos);

	uint hashIdx = computeHashIndex(shortBlockPos);
	while (true) 
	{
		const struct NuiKinfuHashEntry hashEntry = d_hashEntry[hashIdx];
		if(hashEntry.pos[0] == shortBlockPos.x && hashEntry.pos[1] == shortBlockPos.y && hashEntry.pos[2] == shortBlockPos.z && hashEntry.ptr >= 0)
		{
			return hashEntry.ptr * SDF_BLOCK_SIZE3 + linearIdx;
		}
		if (hashEntry.offset < 1)
			break;
		hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
	}
	return -1;
}

inline static float3 sdfInterp(const float3 p1, const float3 p2,
							const float valp1, const float valp2
							)
{
	if (fabs(0.0f - valp1) < 0.00001f) return p1;
	if (fabs(0.0f - valp2) < 0.00001f) return p2;
	if (fabs(valp1 - valp2) < 0.00001f) return p1;

	return p1 + ((0.0f - valp1) / (valp2 - valp1)) * (p2 - p1);
}

__kernel void meshHashScene_kernel(
            __global float*					d_vmap,
			__global float*					d_cmap,
			__global volatile int*				vertex_id,
			__global	int*					d_visibleEntryIDs,
			__global struct NuiKinfuHashEntry*	d_hashEntry,
			__global struct NuiKinfuVoxel*		d_SDFBlocks,
			__constant int*						c_edgeTable,
			__constant int*						c_triTable,
			const		float					virtualVoxelSize)
{
	const uint gid = get_global_id(0);
	const int entryID = d_visibleEntryIDs[gid];
	if(entryID < 0)
		return;
	const struct NuiKinfuHashEntry hashEntry = d_hashEntry[entryID];
	if (hashEntry.ptr < 0)
		return;

	int3 globalPos = (int3)(hashEntry.pos[0], hashEntry.pos[1], hashEntry.pos[2]) * SDF_BLOCK_SIZE;

	for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
	{
		int3 virtualPos = globalPos + (int3)(x, y, z);

		float sdfVals[8];
		float3 points[8];
		float3 colors[8];
		// 0, 0, 0
		int3 localBlockLoc = virtualPos + (int3)(0, 0, 0);
		int blockIndex = findVoxelBlockId(localBlockLoc, d_hashEntry);
		if(blockIndex < 0) continue;
		sdfVals[0] = convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		points[0] = convert_float3(localBlockLoc);
		colors[0] = (float3)(d_SDFBlocks[blockIndex].color[0], d_SDFBlocks[blockIndex].color[1], d_SDFBlocks[blockIndex].color[2]) / 255.f;
		// 1, 0, 0
		localBlockLoc = virtualPos + (int3)(1, 0, 0);
		blockIndex = findVoxelBlockId(localBlockLoc, d_hashEntry);
		if(blockIndex < 0) continue;
		sdfVals[1] = convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		points[1] = convert_float3(localBlockLoc);
		colors[1] = (float3)(d_SDFBlocks[blockIndex].color[0], d_SDFBlocks[blockIndex].color[1], d_SDFBlocks[blockIndex].color[2]) / 255.f;
		// 1, 1, 0
		localBlockLoc = virtualPos + (int3)(1, 1, 0);
		blockIndex = findVoxelBlockId(localBlockLoc, d_hashEntry);
		if(blockIndex < 0) continue;
		sdfVals[2] = convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		points[2] = convert_float3(localBlockLoc);
		colors[2] = (float3)(d_SDFBlocks[blockIndex].color[0], d_SDFBlocks[blockIndex].color[1], d_SDFBlocks[blockIndex].color[2]) / 255.f;
		// 0, 1, 0
		localBlockLoc = virtualPos + (int3)(0, 1, 0);
		blockIndex = findVoxelBlockId(localBlockLoc, d_hashEntry);
		if(blockIndex < 0) continue;
		sdfVals[3] = convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		points[3] = convert_float3(localBlockLoc);
		colors[3] = (float3)(d_SDFBlocks[blockIndex].color[0], d_SDFBlocks[blockIndex].color[1], d_SDFBlocks[blockIndex].color[2]) / 255.f;
		// 0, 0, 1
		localBlockLoc = virtualPos + (int3)(0, 0, 1);
		blockIndex = findVoxelBlockId(localBlockLoc, d_hashEntry);
		if(blockIndex < 0) continue;
		sdfVals[4] = convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		points[4] = convert_float3(localBlockLoc);
		colors[4] = (float3)(d_SDFBlocks[blockIndex].color[0], d_SDFBlocks[blockIndex].color[1], d_SDFBlocks[blockIndex].color[2]) / 255.f;
		// 1, 0, 1
		localBlockLoc = virtualPos + (int3)(1, 0, 1);
		blockIndex = findVoxelBlockId(localBlockLoc, d_hashEntry);
		if(blockIndex < 0) continue;
		sdfVals[5] = convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		points[5] = convert_float3(localBlockLoc);
		colors[5] = (float3)(d_SDFBlocks[blockIndex].color[0], d_SDFBlocks[blockIndex].color[1], d_SDFBlocks[blockIndex].color[2]) / 255.f;
		// 1, 1, 1
		localBlockLoc = virtualPos + (int3)(1, 1, 1);
		blockIndex = findVoxelBlockId(localBlockLoc, d_hashEntry);
		if(blockIndex < 0) continue;
		sdfVals[6] = convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		points[6] = convert_float3(localBlockLoc);
		colors[6] = (float3)(d_SDFBlocks[blockIndex].color[0], d_SDFBlocks[blockIndex].color[1], d_SDFBlocks[blockIndex].color[2]) / 255.f;
		// 0, 1, 1
		localBlockLoc = virtualPos + (int3)(0, 1, 1);
		blockIndex = findVoxelBlockId(localBlockLoc, d_hashEntry);
		if(blockIndex < 0) continue;
		sdfVals[7] = convert_float(d_SDFBlocks[blockIndex].sdf) / 32767.0f;
		points[7] = convert_float3(localBlockLoc);
		colors[7] = (float3)(d_SDFBlocks[blockIndex].color[0], d_SDFBlocks[blockIndex].color[1], d_SDFBlocks[blockIndex].color[2]) / 255.f;

		int cubeIndex = 0;
		if (sdfVals[0] < 0) cubeIndex |= 1; if (sdfVals[1] < 0) cubeIndex |= 2;
		if (sdfVals[2] < 0) cubeIndex |= 4; if (sdfVals[3] < 0) cubeIndex |= 8;
		if (sdfVals[4] < 0) cubeIndex |= 16; if (sdfVals[5] < 0) cubeIndex |= 32;
		if (sdfVals[6] < 0) cubeIndex |= 64; if (sdfVals[7] < 0) cubeIndex |= 128;

		// read number of vertices from texture
        int edgeIndex = vload(cubeIndex, c_edgeTable);
		if(0 == edgeIndex)
			continue;

		float3 vertList[12];
		float3 colorList[12];
		if (edgeIndex & 1)
		{
			vertList[0] = sdfInterp(points[0], points[1], sdfVals[0], sdfVals[1]);
			colorList[0] = sdfInterp(colors[0], colors[1], sdfVals[0], sdfVals[1]);
		}
		if (edgeIndex & 2)
		{
			vertList[1] = sdfInterp(points[1], points[2], sdfVals[1], sdfVals[2]);
			colorList[1] = sdfInterp(colors[1], colors[2], sdfVals[1], sdfVals[2]);
		}
		if (edgeIndex & 4)
		{
			vertList[2] = sdfInterp(points[2], points[3], sdfVals[2], sdfVals[3]);
			colorList[2] = sdfInterp(colors[2], colors[3], sdfVals[2], sdfVals[3]);
		}
		if (edgeIndex & 8)
		{
			vertList[3] = sdfInterp(points[3], points[0], sdfVals[3], sdfVals[0]);
			colorList[3] = sdfInterp(colors[3], colors[0], sdfVals[3], sdfVals[0]);
		}
		if (edgeIndex & 16)
		{
			vertList[4] = sdfInterp(points[4], points[5], sdfVals[4], sdfVals[5]);
			colorList[4] = sdfInterp(colors[4], colors[5], sdfVals[4], sdfVals[5]);
		}
		if (edgeIndex & 32)
		{
			vertList[5] = sdfInterp(points[5], points[6], sdfVals[5], sdfVals[6]);
			colorList[5] = sdfInterp(colors[5], colors[6], sdfVals[5], sdfVals[6]);
		}
		if (edgeIndex & 64)
		{
			vertList[6] = sdfInterp(points[6], points[7], sdfVals[6], sdfVals[7]);
			colorList[6] = sdfInterp(colors[6], colors[7], sdfVals[6], sdfVals[7]);
		}
		if (edgeIndex & 128)
		{
			vertList[7] = sdfInterp(points[7], points[4], sdfVals[7], sdfVals[4]);
			colorList[7] = sdfInterp(colors[7], colors[4], sdfVals[7], sdfVals[4]);
		}
		if (edgeIndex & 256)
		{
			vertList[8] = sdfInterp(points[0], points[4], sdfVals[0], sdfVals[4]);
			colorList[8] = sdfInterp(colors[0], colors[4], sdfVals[0], sdfVals[4]);
		}
		if (edgeIndex & 512)
		{
			vertList[9] = sdfInterp(points[1], points[5], sdfVals[1], sdfVals[5]);
			colorList[9] = sdfInterp(colors[1], colors[5], sdfVals[1], sdfVals[5]);
		}
		if (edgeIndex & 1024)
		{
			vertList[10] = sdfInterp(points[2], points[6], sdfVals[2], sdfVals[6]);
			colorList[10] = sdfInterp(colors[2], colors[6], sdfVals[2], sdfVals[6]);
		}
		if (edgeIndex & 2048)
		{
			vertList[11] = sdfInterp(points[3], points[7], sdfVals[3], sdfVals[7]);
			colorList[11] = sdfInterp(colors[3], colors[7], sdfVals[3], sdfVals[7]);
		}

		for (int i = 0; i < 6 ; i += 3)
        {
			int table_index = mul24(cubeIndex, 16) + i;
			int v1 = vload(table_index    , c_triTable);
			int v2 = vload(table_index + 1, c_triTable);
			int v3 = vload(table_index + 2, c_triTable);
			if(v1 < 0 || v1 >= 12)
				break;

			int current_id = atomic_add(vertex_id, 3);
			if(current_id > MAX_OUTPUT_VERTEX_SIZE)
			{
				break;
			}

			vstore3(vertList[v1] * virtualVoxelSize, current_id + i   , d_vmap);
			vstore3(vertList[v2] * virtualVoxelSize, current_id + i + 1, d_vmap);
			vstore3(vertList[v3] * virtualVoxelSize, current_id + i + 2, d_vmap);

			vstore4((float4)(colorList[v1], 1.0f), current_id + i   , d_cmap);
			vstore4((float4)(colorList[v2], 1.0f), current_id + i + 1, d_cmap);
			vstore4((float4)(colorList[v3], 1.0f), current_id + i + 2, d_cmap);
		}
	}
}