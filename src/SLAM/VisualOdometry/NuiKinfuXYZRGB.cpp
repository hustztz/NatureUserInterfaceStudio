#include "NuiKinfuXYZRGB.h"
#include "NuiKinfuVertexCache.h"


NuiKinfuXYZRGB::NuiKinfuXYZRGB()
{
}

NuiKinfuXYZRGB::~NuiKinfuXYZRGB()
{
	clear();
}

bool	NuiKinfuXYZRGB::incrementPoints(NuiKinfuVertexCache* pVertexCache)
{
	if (!pVertexCache)
		return false;

	const int inputSize = pVertexCache->pointSize();
	if (0 == inputSize)
		return false;

	const SgVec4f* inputColors = pVertexCache->getColors();
	const SgVec3f* inputVertices = pVertexCache->getVertices();
	if (!inputVertices)
		return false;

	const int nSize = pointSize() + inputSize;
	resizePoints(nSize);

	int i = 0;
	writeLock();
	pVertexCache->readLock();
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
	for (auto iter = m_pointCloud.begin(); iter != m_pointCloud.end(); iter ++, i ++)
	{
		iter->x = inputVertices[i][0];
		iter->y = inputVertices[i][1];
		iter->z = inputVertices[i][2];
		if (inputColors)
		{
			iter->r = (unsigned char)(inputColors[i][0] * 255);
			iter->g = (unsigned char)(inputColors[i][1] * 255);
			iter->b = (unsigned char)(inputColors[i][2] * 255);
			iter->a = 255;
		}
		else
		{
			iter->rgba = 0;
		}
	}
	pVertexCache->readUnlock();
	writeUnlock();

	return true;
}
