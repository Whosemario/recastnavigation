#pragma once

#include "Sample_TileMesh.h"
#include <memory>
#include <functional>
#include <cstdlib>
#include <vector>
#include <set>

using morton_code_t = std::uint64_t;

struct FloatVector {
public:
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;

public:
	FloatVector() = default;
	FloatVector(float inX, float inY, float inZ) :
		x(inX), y(inY), z(inZ) {}
	FloatVector(float inF):
		x(inF), y(inF), z(inF) {}

	FloatVector(FloatVector&) = default;

	FloatVector& operator=(const FloatVector&) = default;

	inline FloatVector operator+(const FloatVector& other) const {
		return FloatVector(x + other.x, y + other.y, z + other.z);
	}

	inline FloatVector operator-(const FloatVector& other) const {
		return FloatVector(x - other.x, y - other.y, z - other.z);
	}

	inline float cos(const FloatVector& other) const {
		return x * other.x + y * other.y + z * other.z;
	}

	inline FloatVector operator*(float a) const {
		return FloatVector(x * a, y * a, z *a);
	}
};

struct FloatTri {
public:
	FloatVector indexes[3];
	FloatVector normal;
};

using GetVolumeBoudingBoxFunc = std::function<bool(FloatVector&, FloatVector&)>;
using OverlapBoxBlockingTestFunc = std::function<bool(const FloatVector&, float, int)>;

class SVONVolume
{
public:
	SVONVolume(int, GetVolumeBoudingBoxFunc&&, OverlapBoxBlockingTestFunc&&);

public:
	bool generate();

	/*
	* vector里面每两个FloatVector一组，记录的是box的最低最高三维点
	*/
	void getAllBlockedBox(std::vector<FloatVector>&) const;

private:
	void setupVolume() {
		boundFunc_(origin_, extent_);
	}

	/*
	* 从 Layer1 到 LaynerN 标记某个体素 是否有 物体
	* 这样预处理是为了加速后续的计算
	*/
	bool buildBlockedIndices();

	/*
	* (2^(voxelPower_-layer))^3
	*/
	inline std::int32_t getNodesInLayer(std::int32_t layer) const {
		return static_cast<std::int32_t>(powf(powf(2, voxelPower_ - layer), 3));
	}

	/*
	* extent.X * 2 / 2 ^ (voxelPower_ - layer)
	*/
	inline float getVoxelSize(std::int32_t layer) const {
		return extent_.x * 2.0f / powf(2, voxelPower_ - layer);
	}

	void getPositionFromMortonCode(morton_code_t, FloatVector&, std::int32_t) const;

private:
	int voxelPower_;
	GetVolumeBoudingBoxFunc boundFunc_;
	OverlapBoxBlockingTestFunc checkFunc_;

	FloatVector origin_;
	FloatVector extent_;

	std::vector<std::set<morton_code_t>> blockedIndices_;

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
	bool overlapBoxAndTri(const FloatVector&, float, const FloatTri&);

private:
	FloatVector origin_;
	FloatVector extent_;

	bool hasBuild_ = false;
	SVONVolumePtr vol_;
};
