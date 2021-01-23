#pragma once

#include "Sample_TileMesh.h"
#include <memory>
#include <functional>

struct FloatVector {
public:
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;

public:
	FloatVector() {}
	FloatVector(float inX, float inY, float inZ) :
		x(inX), y(inY), z(inZ) {}
	FloatVector(float inF):
		x(inF), y(inF), z(inF) {}
};

using GetVolumeBoudingBoxFunc = std::function<bool(FloatVector&, FloatVector&)>;
using OverlapBoxBlockingTestFunc = std::function<bool(const FloatVector&, float, int)>;

class SVONVolume
{
public:
	SVONVolume(int, GetVolumeBoudingBoxFunc&&, OverlapBoxBlockingTestFunc&&);

public:
	bool generate();

	void setupVolume() {
		boundFunc_(origin_, extent_);
	}
	
private:
	int voxelPowner_;
	GetVolumeBoudingBoxFunc boundFunc_;
	OverlapBoxBlockingTestFunc checkFunc_;

	FloatVector origin_;
	FloatVector extent_;

};
using SVONVolumePtr = std::shared_ptr<SVONVolume>;

class Sample_SVOMesh : public Sample_TileMesh
{
public:
	virtual void handleRender() override;
	virtual bool handleBuild() override;

private:
	
	bool getVolumeBoundingBox(FloatVector& o, FloatVector& e) {
		o = origin_;
		e = extent_;
		return true;
	}
	bool overlapBoxBlockingText(const FloatVector&, float, int);

private:
	FloatVector origin_;
	FloatVector extent_;

	bool hasBuild_ = false;
	SVONVolumePtr vol_;
};
