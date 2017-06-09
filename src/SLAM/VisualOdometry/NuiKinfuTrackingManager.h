#pragma once

#include "Foundation\NuiThreadObject.h"
#include "NuiKinfuTrackingEngine.h"
#include "Frame/Buffer/NuiVisualFrameCircleBuffer.h"

#include <boost/thread/mutex.hpp>

class NuiKinfuScene;

namespace NuiKinfuEngine
{
	class NuiKinfuTrackingManager : public NuiThreadObject
	{
	public:
		NuiKinfuTrackingManager();
		virtual ~NuiKinfuTrackingManager();

		virtual void	reset() override;

		bool	pushbackFrame(std::shared_ptr<NuiVisualFrame> pFrame) { return m_buffer.pushbackVisualFrame(pFrame); }
		size_t	getLagFrames() { return m_buffer.size(); }
		void	setScene(NuiKinfuScene* pScene) { m_pScene = pScene; }
		void	evaluateCLData(NuiCLMappableData* pCLData);
		void	log(const std::string& fileName) const { m_trackingEngine.log(fileName); }
		boost::mutex& getMutex() { return m_trackingMutex; }

		void	setColorTracker(bool bHasColor) { m_trackingConfig.bHasColor = bHasColor; }
		void	setTranslateBasis(const Vector3f& basis);
		const Vector3f&	getTranslateBasis() const { return m_trackingEngine.getTranslateBasis(); }
		void	setIntegrationMetricThreshold(float threshold);
		float	getTrackerError() const { return m_trackingEngine.getTrackerError(); }
		int		getFrameID() const { return m_trackingEngine.getFrameID(); }
		const NuiCameraPos&		getLatestCameraPose() const { return m_trackingEngine.getCameraPose();	}

	private:
		virtual bool process() override;

	private:
		NuiTrackerConfig					m_trackingConfig;
		NuiKinfuTrackingEngine				m_trackingEngine;
		boost::mutex						m_trackingMutex;
		NuiKinfuScene*						m_pScene;

		NuiVisualFrameCircleBuffer			m_buffer;
		bool								m_bAutoReset;
	};
}