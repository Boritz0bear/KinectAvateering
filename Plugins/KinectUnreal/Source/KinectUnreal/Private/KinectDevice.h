// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <CoreUObject.h>
#include <Vector.h>
#include <Kinect.h>
#include <kinect.Face.h>
#include <Runtime/Engine/Classes/Engine/Texture2D.h>
#include "KinectDTO.h"
#include <mutex>

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease) {
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

/**
 *
 */
class KinectDevice : public FRunnable
{
public:
	/* Begin FRunnable interface. */
	virtual bool	Init()	override;
	virtual uint32	Run()	override;
	virtual void	Stop()	override;
	/* End FRunnable interface */

	/* Begin Singleton Interface*/
	static KinectDevice* const GetInstance();
	static void DeleteInstance();
	/* End Singleton Interface*/

	DepthSpacePoint*		getDepthCoordinates() const;
	CameraIntrinsics		getCameraIntrinsics() const;
	TArray<Body20>			getBodies() const;
	Body20					getBody(int idx) const;
	void					setIsUpdatingColorCamera(bool value);

	UTexture2D*				getColorTexture();

private:
	KinectDevice();
	~KinectDevice();

	// Thêm mutex để bảo vệ các dữ liệu chia sẻ
	std::mutex kinectDataMutex;

	void loadBody(Body20& body, IBody* pBody, UINT16* pDepthBuffer);
	void copyBufferToTexture(TWeakObjectPtr<UTexture2D> Texture, unsigned char * pData, int width, int height, int numColors);

	// Sử dụng unique_ptr để quản lý bộ nhớ
	std::unique_ptr<IKinectSensor>				m_pKinectSensor;
	std::unique_ptr<ICoordinateMapper>			m_pCoordinateMapper;
	std::unique_ptr<ColorSpacePoint[]>			m_pColorCoordinates;
	std::unique_ptr<ColorSpacePoint[]>			m_pColorCoordinatesAux;
	std::unique_ptr<DepthSpacePoint[]>			m_pDepthCoordinates;

	// Frame readers
	//IMultiSourceFrameReader*	m_pMultiSourceFrameReader;
	std::unique_ptr<IDepthFrameReader>			m_pDepthFrameReader;
	std::unique_ptr<IColorFrameReader>			m_pColorFrameReader;
	std::unique_ptr<IBodyFrameReader>			m_pBodyFrameReader;
	std::unique_ptr<IBodyIndexFrameReader>		m_pBodyIndexFrameReader;
	std::unique_ptr<IInfraredFrameReader>		m_pInfraredFrameReader;

	//Face Tracking
	bool							extract2DFaces;
	std::unique_ptr<IFaceFrameSource*[]>		m_pFaceFrameSources;
	std::unique_ptr<IFaceFrameReader*[]>		m_pfaceFrameReaders;
	TArray<UINT32>					faceTriangles;
	int								totalFaceVertices;


	//HD Face Tracking
	std::unique_ptr<IHighDefinitionFaceFrameReader*[]>	m_pFaceFrameReader;
	std::unique_ptr<IHighDefinitionFaceFrameSource*[]> m_pHDFaceFrameSource;
	std::unique_ptr<IFaceAlignment*[]>					m_pFaceAlignment;
	std::unique_ptr<IFaceModel*[]>						m_pFaceModel;
	std::unique_ptr<IFaceModelBuilder*[]>				m_pFaceModelBuilder;
	TArray<CameraSpacePoint>		m_pFaceVertices[KinectDTO::cTotalBodies];

	// Frame data
	std::unique_ptr<RGBQUAD[]>            m_pColorRGBX;
	std::unique_ptr<BYTE[]>				infraRedBuffer;
	std::unique_ptr<UINT16[]>				m_pInfraRedBuffer;
	//TODO: Remove this
	std::unique_ptr<UINT16[]>				m_pDethRawBuffer;


	TWeakObjectPtr<UTexture2D> 				cameraTexture;
	TWeakObjectPtr<UTexture2D> 				IRTexture;

	KinectDTO*			kinectData;
	DepthRawData20*		depthRawData;
	DepthRawData20		tempDepthData[KinectDTO::cDepthWidth * KinectDTO::cDepthHeight];

	/* Frunnable Helper. */
	bool						bRunning;
	static KinectDevice*		Instance;
	FRunnableThread*			Thread;
	FRWLock						frwLock;

	CameraIntrinsics		cameraIntrinsics;
	bool					paused;
	FString					deviceId;
	WAITABLE_HANDLE			waitableHandle;
	BOOLEAN					lastAvailable;

	bool				extractColorData;
	bool				extractIRData;
	bool				extractSkeletonData;
	bool				extractDepthCoordinates;
	bool				extractFaces;
	bool				extractDepthData;
	bool				isUpdatingColorFeed;

};

