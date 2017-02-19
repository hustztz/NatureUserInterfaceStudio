#pragma once

class NuiKinfuFrameImpl;
class NuiKinfuTransform;

class NuiKinfuTracker
{
public:
	virtual bool	trackFrame(NuiKinfuFrameImpl* pFrame, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint) = 0;
	virtual bool	readFrame(NuiKinfuFrameImpl* pFrame) = 0;
	virtual void	transformPrevsFrame(NuiKinfuTransform* pTransform) = 0;
	virtual void	resizePrevsFrame() = 0;
	virtual void	copyPrevsFrame() = 0;

	virtual bool	hasColorData() const { return false; }
	virtual bool	log(const std::string& fileName) const { return false; }
	virtual float	getError() const { return 0.0; }
	virtual float	getCount() const { return 0.0; }
};