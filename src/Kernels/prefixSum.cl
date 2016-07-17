
//Must be a power of two
#define WORKGROUP_SIZE 256
#define LOG2_WARP_SIZE 5U
#define WARP_SIZE (1U << LOG2_WARP_SIZE)

//Almost the same as naive scan1Inclusive but doesn't need barriers
//and works only for size <= WARP_SIZE
inline uint warpScanInclusive(uint idata, volatile __local uint *l_Data, uint size)
{
    uint pos = 2 * get_local_id(0) - (get_local_id(0) & (size - 1));
    l_Data[pos] = 0;
    pos += size;
    l_Data[pos] = idata;

    if(size >=  2) l_Data[pos] += l_Data[pos -  1];
    if(size >=  4) l_Data[pos] += l_Data[pos -  2];
    if(size >=  8) l_Data[pos] += l_Data[pos -  4];
    if(size >= 16) l_Data[pos] += l_Data[pos -  8];
    if(size >= 32) l_Data[pos] += l_Data[pos - 16];

    return l_Data[pos];
}

inline uint warpScanExclusive(uint idata, __local uint *l_Data, uint size)
{
    return warpScanInclusive(idata, l_Data, size) - idata;
}

inline uint scan1Inclusive(uint idata, __local uint *l_Data, uint size)
{
    if(size > WARP_SIZE){
        //Bottom-level inclusive warp scan
        uint warpResult = warpScanInclusive(idata, l_Data, WARP_SIZE);

        //Save top elements of each warp for exclusive warp scan
        //sync to wait for warp scans to complete (because l_Data is being overwritten)
        barrier(CLK_LOCAL_MEM_FENCE);
        if( (get_local_id(0) & (WARP_SIZE - 1)) == (WARP_SIZE - 1) )
            l_Data[get_local_id(0) >> LOG2_WARP_SIZE] = warpResult;

        //wait for warp scans to complete
        barrier(CLK_LOCAL_MEM_FENCE);
        if( get_local_id(0) < (WORKGROUP_SIZE / WARP_SIZE) ){
            //grab top warp elements
            uint val = l_Data[get_local_id(0)];
            //calculate exclsive scan and write back to shared memory
            l_Data[get_local_id(0)] = warpScanExclusive(val, l_Data, size >> LOG2_WARP_SIZE);
        }

        //return updated warp scans with exclusive scan results
        barrier(CLK_LOCAL_MEM_FENCE);
        return warpResult + l_Data[get_local_id(0) >> LOG2_WARP_SIZE];
    }else{
        return warpScanInclusive(idata, l_Data, size);
    }
}

inline uint scan1Exclusive(uint idata, __local uint *l_Data, uint size)
{
    return scan1Inclusive(idata, l_Data, size) - idata;
}

//Vector scan: the array to be scanned is stored
//in work-item private memory as uint4
inline uint4 scan4Inclusive(uint4 data4, __local uint *l_Data, uint size)
{
    //Level-0 inclusive scan
    data4.y += data4.x;
    data4.z += data4.y;
    data4.w += data4.z;

    //Level-1 exclusive scan
    uint val = scan1Inclusive(data4.w, l_Data, size / 4) - data4.w;

    return (data4 + (uint4)val);
}

inline uint4 scan4Exclusive(uint4 data4, __local uint *l_Data, uint size)
{
    return scan4Inclusive(data4, l_Data, size) - data4;
}

////////////////////////////////////////////////////////////////////////////////
// Scan kernels
////////////////////////////////////////////////////////////////////////////////
__kernel __attribute__((reqd_work_group_size(WORKGROUP_SIZE, 1, 1)))
void scanExclusiveLocal1(
    __global uint4 *d_Src,
    __global uint4 *d_Dst,
    __local uint *l_Data,
    uint size
	)
{
    //Load data
    uint4 idata4 = d_Src[get_global_id(0)];

    //Calculate exclusive scan
    uint4 odata4  = scan4Exclusive(idata4, l_Data, size);

    //Write back
    d_Dst[get_global_id(0)] = odata4;
}

//Exclusive scan of top elements of bottom-level scans (4 * THREADBLOCK_SIZE)
__kernel __attribute__((reqd_work_group_size(WORKGROUP_SIZE, 1, 1)))
void scanExclusiveLocal2(
    __global uint *d_Src,
    __global uint *d_Buf,
    __global uint *d_Dst,
    __local uint *l_Data,
    uint size
	)
{
    //Load top elements
    //Convert results of bottom-level scan back to inclusive
    //Skip loads and stores for inactive work-items of the work-group with highest index(pos >= N)
    uint data = 0;
    if(get_global_id(0) < size)
    data =
        d_Dst[(4 * WORKGROUP_SIZE - 1) + (4 * WORKGROUP_SIZE) * get_global_id(0)] + 
        d_Src[(4 * WORKGROUP_SIZE - 1) + (4 * WORKGROUP_SIZE) * get_global_id(0)];

    //Compute
    uint odata = scan1Exclusive(data, l_Data, size);

    //Avoid out-of-bound access
    if(get_global_id(0) < size)
        d_Buf[get_global_id(0)] = odata;
}

//Final step of large-array scan: combine basic inclusive scan with exclusive scan of top elements of input arrays
__kernel __attribute__((reqd_work_group_size(WORKGROUP_SIZE, 1, 1)))
void uniformUpdate(
    __global uint *d_Buf,
    __global uint4 *d_Data
	)
{
    __local uint buf[1];

    uint4 data4 = d_Data[get_global_id(0)];

    if(get_local_id(0) == 0)
        buf[0] = d_Buf[get_group_id(0)];

    barrier(CLK_LOCAL_MEM_FENCE);
    data4 += (uint4)buf[0];
    d_Data[get_global_id(0)] = data4;
}