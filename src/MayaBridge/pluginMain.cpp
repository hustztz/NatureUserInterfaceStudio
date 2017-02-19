#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MOpenCLInfo.h>
#include <maya/MDrawRegistry.h>

#include "MayaBridge/PointCloudShape/NuiMayaMappableData.h"
#include "MayaBridge/PointCloudShape/NuiMayaPointCloudShape.h"
#include "MayaBridge/PointCloudShape/NuiMayaPointCloudGeometryOverride.h"
#include "MayaBridge/PointCloudProcess/NuiMayaMeshData.h"
#include "MayaBridge/PointCloudProcess/NuiMayaMeshGenerator.h"
#include "MayaBridge/SkeletonDriver/NuiMayaSkeletonData.h"
#include "MayaBridge/SkeletonDriver/NuiMayaSkeletonDriver.h"
#include "MayaBridge/SkeletonDriver/NuiMayaImageData.h"
#include "MayaBridge/SkeletonDriver/NuiMayaFacialModelData.h"
#include "MayaBridge/SkeletonDriver/NuiMayaFacialModelDriver.h"
#include "MayaBridge/SkeletonDriver/NuiMayaGestureData.h"
#include "MayaBridge/DeviceGrabber/NuiMayaDeviceGrabber.h"
#include "MayaBridge/DeviceGrabber/NuiMayaPreviewerRenderOverride.h"
#include "MayaBridge/NuiMayaProfiler.h"
#include "MayaBridge/HardwareUtilities/NuiMayaHWMappable.h"
#include "MayaBridge/HardwareUtilities/NuiMayaOpenGLThreadProvider.h"
#include "MayaBridge/HardwareUtilities/NuiMayaOpenCLUtilities.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiOpenCLBufferFactory.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"
#include "Foundation/NuiDebugMacro.h"

#define	PLUGIN_VENDOR "Tingzhu"
#define	PLUGIN_VERSION "2015"

#define NUI_KERNEL_DEV_MODE "NUI_ENABLE_KERNEL_DEV_MODE"

static MString sDrawDbClassification("drawdb/geometry/natureUserInterface");
static MString sDrawRegistrantId("NatureUserInterfacePlugin");

static NuiMayaPreviewerRenderOverride* sPreviewerRenderOverrideInstance = NULL;

MStatus uninitializePlugin( MObject obj);

MStatus initializePlugin( MObject obj )
//
//	Description:
//		this method is called when the plug-in is loaded into Maya.  It 
//		registers all of the services that this plug-in provides with 
//		Maya.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{
	MStatus status;
	MFnPlugin plugin( obj, PLUGIN_VENDOR, PLUGIN_VERSION, "Any");

	status = plugin.registerData( "pointCloudData", NuiMayaMappableData::id,
		&NuiMayaMappableData::creator,
		MPxData::kGeometryData );
	if ( ! status ) {
		cerr << "Failed to register geometry data : pointCloudData \n";
		uninitializePlugin(obj);
		return status;
	}

	status = plugin.registerShape( "pointCloudShape", NuiMayaPointCloudShape::id,
		&NuiMayaPointCloudShape::creator,
		&NuiMayaPointCloudShape::initialize,
		NULL,
		&sDrawDbClassification );
	if ( ! status ) {
		cerr << "Failed to register shape : pointCloudShape\n";
		uninitializePlugin(obj);
		return status;
	}

	status = MHWRender::MDrawRegistry::registerGeometryOverrideCreator(
		sDrawDbClassification,
		sDrawRegistrantId,
		NuiMayaPointCloudGeometryOverride::Creator);
	if (!status) {
		status.perror("Failed to register Viewport 2.0 geometry override\n");
		uninitializePlugin(obj);
		return status;
	}

	status = plugin.registerData( "deviceMeshData", NuiMayaMeshData::id,
		&NuiMayaMeshData::creator,
		MPxData::kData );
	if ( ! status ) {
		cerr << "Failed to register geometry data : deviceMeshData \n";
		uninitializePlugin(obj);
		return status;
	}

	status = plugin.registerNode( "meshGenerator", NuiMayaMeshGenerator::id, NuiMayaMeshGenerator::creator,
		NuiMayaMeshGenerator::initialize );
	if (!status) {
		status.perror("registerNode meshGenerator");
		uninitializePlugin(obj);
		return status;
	}

	status = plugin.registerData( "gestureData", NuiMayaGestureData::id,
		&NuiMayaGestureData::creator,
		MPxData::kData );
	if ( ! status ) {
		cerr << "Failed to register geometry data : gestureData \n";
		uninitializePlugin(obj);
		return status;
	}

	status = plugin.registerData( "skeletonData", NuiMayaSkeletonData::id,
		&NuiMayaSkeletonData::creator,
		MPxData::kData );
	if ( ! status ) {
		cerr << "Failed to register geometry data : skeletonData \n";
		uninitializePlugin(obj);
		return status;
	}

	status = plugin.registerNode( "skeletonDriver", NuiMayaSkeletonDriver::id, NuiMayaSkeletonDriver::creator,
		NuiMayaSkeletonDriver::initialize );
	if (!status) {
		status.perror("registerNode: skeleton driver node");
		uninitializePlugin(obj);
		return status;
	}

	status = plugin.registerData( "imageData", NuiMayaImageData::id,
		&NuiMayaImageData::creator,
		MPxData::kData );
	if ( ! status ) {
		cerr << "Failed to register geometry data : imageData \n";
		uninitializePlugin(obj);
		return status;
	}

	status = plugin.registerData( "facialModelData", NuiMayaFacialModelData::id,
		&NuiMayaFacialModelData::creator,
		MPxData::kData );
	if ( ! status ) {
		cerr << "Failed to register geometry data : facialModelData \n";
		uninitializePlugin(obj);
		return status;
	}

	status = plugin.registerNode( "facialModelDriver", NuiMayaFacialModelDriver::id, NuiMayaFacialModelDriver::creator,
		NuiMayaFacialModelDriver::initialize );
	if (!status) {
		status.perror("registerNode: facial model driver node");
		uninitializePlugin(obj);
		return status;
	}

	status = plugin.registerNode( "natureUserInterfaceGrabber", NuiMayaDeviceGrabber::id, NuiMayaDeviceGrabber::creator,
		NuiMayaDeviceGrabber::initialize );
	if (!status) {
		status.perror("register Grabber Node");
		uninitializePlugin(obj);
		return status;
	}

	// Register override
	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (renderer)
	{
		if (!sPreviewerRenderOverrideInstance)
		{
			sPreviewerRenderOverrideInstance = new NuiMayaPreviewerRenderOverride( NuiMayaPreviewerRenderOverride::kNuiPreviewerRendererName );
			renderer->registerOverride(sPreviewerRenderOverrideInstance);
		}
	}

	// set the mel procs to be run when the plugin is loaded / unloaded
	if (MGlobal::mayaState() == MGlobal::kInteractive) {
		// We have to explicitly source this script here. In
		// particular, it can't be done from the UI creation script
		// below. The UI creation script is executed by
		// TloadPluginAction through a deferred evaluation. The
		// deferred evaluation can cause the UI deletion script to be
		// executed before the creation UI one!
		status = MGlobal::executeCommand("source NatureUserInterfaceMenu");
		if (!status) {
			status.perror("NatureUserInterfaceInitUI");
			return status;
		}
	}

	NuiMayaProfiler::assignFuncPtr();
	NuiMayaProfiler::addNuiCategory("NatureUserInterface");

	// Kernel dev mode, when it is on, it will rebuild kernels at certain time
	// like when tool context activated. Using this is to facilitate the kernel develop
	// work, we don't need to restart or even rebuild maya to make new kernel change
	// take effect.
	// Of source, rebuilding kernel too often may have performance penalty, so please
	// use it when during kernel development work only.
	{
		const bool enableKernelDevMode = getenv(NUI_KERNEL_DEV_MODE) != nullptr;
		NuiOpenCLGlobal::instance().kernelDevMode(enableKernelDevMode);
		if (enableKernelDevMode) {
			NUI_DEBUG("Kernel develop mode is on.\n");
		}
		else {
			NUI_DEBUG("Kernel develop mode is off.\n");
		}
	}

	bool needCLProfiling = false;
#ifdef _DEBUG
	needCLProfiling = true;
#endif
	// init globals to core
	if (MOpenCLInfo::getOpenCLContext()) {
		NuiOpenCLGlobal::instance().setupOpenCL(
			MOpenCLInfo::getOpenCLContext(),
			MOpenCLInfo::getOpenCLDeviceId(),
			MOpenCLInfo::createOpenCLCommandQueue(needCLProfiling));

		if (NuiOpenCLGlobal::instance().isCLReady()) {
			NUI_DEBUG("OpenCL initialize successfully.\n");
		}
		else {
			NUI_ERROR("OpenCL initialize failed!!\n");
		}

		NuiOpenCLGlobal::instance().isGL(MHWRender::MRenderer::theRenderer()->drawAPIIsOpenGL());

		// Set kernel dir
		std::string kernelsFolder = MString("$NUI_LOCATION").expandEnvironmentVariablesAndTilde().asChar();
		kernelsFolder += "\\Kernels";
		NuiOpenCLGlobal::instance().kernelDir(kernelsFolder.c_str());

		// Set temp dir
		{
			std::string tmpDir = NuiOpenCLGlobal::instance().prepareTmpDir(kernelsFolder.c_str());
			NUI_DEBUG("TempDir:  %s\n",
				(tmpDir != "") ? NuiOpenCLGlobal::instance().tmpDir().c_str() :
				"Failed to create!");
		}

		// Only build all used opencl kernels once ahead of time here
		// when in debug mode.
		// Set whether debug mode
		NuiOpenCLKernelManager::instance().buildAllShaders();
		
		// Register buffer functions, register more if needed in the future
		NuiOpenCLBufferFactory::RegisterAsUInt32IndexBufferCLFn(
			NuiMayaHWMappable::asUInt32IndexBufferCL);
		NuiOpenCLBufferFactory::RegisterAsPosition3fBufferCLFn(
			NuiMayaHWMappable::asPosition3fBufferCL);
		NuiOpenCLBufferFactory::RegisterAsColor4fBufferCLFn(
			NuiMayaHWMappable::asColor4fBufferCL);
		NuiOpenCLBufferFactory::RegisterAsNormal3fBufferCLFn(
			NuiMayaHWMappable::asNormal3fBufferCL);
		NuiOpenCLBufferFactory::RegisterAsPatchUV2fBufferCLFn(
			NuiMayaHWMappable::asTexture2fBufferCL);
		NuiOpenCLBufferFactory::RegisterAsTexture1fBufferCLFn(
			NuiMayaHWMappable::asTexture1fBufferCL);

		// Register functions for NuiGPUMemManager
		NuiGPUMemManager::RegisterInformRenderHoldGPU(NuiMayaOpenCLUtilities::informMayaHoldGPU);
		NuiGPUMemManager::RegisterInformRenderReleaseGPU(NuiMayaOpenCLUtilities::informMayaReleaseGPU);
		NuiGPUMemManager::RegisterLockRenderResourceHandle(NuiMayaOpenCLUtilities::lockResourceHandle);
		NuiGPUMemManager::RegisterUnlockRenderResourceHandle(NuiMayaOpenCLUtilities::unlockResourceHandle);

	}
	else {
		NUI_WARN("OpenCL is not available, sculpting will not work!\n");
		if (getenv("MAYA_FORCE_DX_WARP") != nullptr) {
			NUI_WARN("\"MAYA_FORCE_DX_WARP\" is set in your environment,"
				"this may cause OpenCL not work.");
		}
	}

	// Initialize the graphics thread for OpenGL
	// The OpenGL context shares resources with Maya's resource widget.
	NuiOpenGLThread::instance().initialize(
		std::make_shared<NuiMayaOpenGLThreadProvider>());

	return status;
}

MStatus uninitializePlugin( MObject obj)
//
//	Description:
//		this method is called when the plug-in is unloaded from Maya. It 
//		deregisters all of the services that it was providing.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{
	// Uninitialize sculpt
	NuiOpenCLGlobal::instance().uninitialize();

	// Shutdown the OpenGL thread
	NuiOpenGLThread::instance().shutdown();

	MStatus   status;
	MFnPlugin plugin( obj );

	status = plugin.deregisterNode( NuiMayaDeviceGrabber::id );
	if ( ! status ) {
		cerr << "Failed to deregister node : Grabber \n";
	}

	status = plugin.deregisterNode( NuiMayaFacialModelDriver::id );
	if ( ! status ) {
		cerr << "Failed to deregister node : FacialModel driver \n";
	}

	status = plugin.deregisterData( NuiMayaFacialModelData::id );
	if ( ! status ) {
		cerr << "Failed to deregister data : facial model Data \n";
	}

	status = plugin.deregisterData( NuiMayaImageData::id );
	if ( ! status ) {
		cerr << "Failed to deregister data : NuiMayaCompositeImageData \n";
	}

	status = plugin.deregisterNode( NuiMayaSkeletonDriver::id );
	if ( ! status ) {
		cerr << "Failed to deregister node : skeleton driver \n";
	}

	status = plugin.deregisterData( NuiMayaSkeletonData::id );
	if ( ! status ) {
		cerr << "Failed to deregister data : skeleton Data \n";
	}

	status = plugin.deregisterData( NuiMayaGestureData::id );
	if ( ! status ) {
		cerr << "Failed to deregister data : gesture Data \n";
	}

	status = plugin.deregisterNode( NuiMayaMeshGenerator::id );
	if ( ! status ) {
		cerr << "Failed to deregister node : meshGenerator \n";
	}

	status = plugin.deregisterData( NuiMayaMeshData::id );
	if ( ! status ) {
		cerr << "Failed to deregister geometry data : deviceMeshData \n";
	}

	status = MHWRender::MDrawRegistry::deregisterGeometryOverrideCreator(
		sDrawDbClassification,
		sDrawRegistrantId);
	if (!status) {
		status.perror("Failed to deregister sub-scene override : pointCloudGeometryOverride \n");
	}

	status = plugin.deregisterNode( NuiMayaPointCloudShape::id );
	if ( ! status ) {
		cerr << "Failed to deregister shape : pointCloudShape \n";
	}

	status = plugin.deregisterData( NuiMayaMappableData::id );
	if ( ! status ) {
		cerr << "Failed to deregister geometry data : pointCloudData \n";
	}

	MHWRender::MRenderer* renderer = MHWRender::MRenderer::theRenderer();
	if (renderer)
	{
		if (sPreviewerRenderOverrideInstance)
		{
			renderer->deregisterOverride(sPreviewerRenderOverrideInstance);
			delete sPreviewerRenderOverrideInstance;
		}
		sPreviewerRenderOverrideInstance = NULL;
	}

	return status;
}
