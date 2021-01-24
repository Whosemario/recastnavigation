#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_SVOMesh.h"

#include "libmorton/morton.h"
#include <cassert>

#define INPUT_VOXEL_POWER 4 

static int ceillog2(int x) {
	static const int log_2[256] = {  /* log_2[i] = ceil(log2(i - 1)) */
	  0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
	  6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
	  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
	  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
	  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8
	};
	int l = 0;
	x--;
	while (x >= 256) { l += 8; x >>= 8; }
	return l + log_2[x];
}

#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#define LINE_WIDTH 2.0f
#define LINE_COLOR_RED duRGBA(255, 0, 0, 128)

SVONVolume::SVONVolume(int voxelPower, GetVolumeBoudingBoxFunc&& f1, OverlapBoxBlockingTestFunc&& f2)
	: voxelPower_(voxelPower), boundFunc_(std::move(f1)), checkFunc_(std::move(f2)) {}

bool SVONVolume::generate()
{
	setupVolume();

	buildBlockedIndices();

	return true;
}

void SVONVolume::getPositionFromMortonCode(morton_code_t mc, FloatVector& pos, std::int32_t layer) const {
	std::uint32_t x, y, z;
	morton3D_64_decode(mc, x, z, y);
	auto sz = getVoxelSize(layer);
	pos = origin_ - extent_ + FloatVector(x * sz, y * sz, z * sz) + FloatVector(sz * 0.5f);
}

bool SVONVolume::buildBlockedIndices()
{
	auto numLayers = voxelPower_ + 1;
	blockedIndices_.clear();

	auto nodesNum = getNodesInLayer(1);

	blockedIndices_.push_back(std::set<morton_code_t>());
	for (int mc = 0; mc < nodesNum; ++mc) {
		FloatVector position;
		getPositionFromMortonCode(mc, position, 1);
		if (checkFunc_(position, getVoxelSize(1) * 0.5f, 1)) {
			blockedIndices_[0].insert(static_cast<morton_code_t>(mc));
		}
	}
	return true;
}

void SVONVolume::getAllBlockedBox(std::vector<FloatVector>& out) const {

	std::int32_t layer = 1;
	for (auto& mc_set : blockedIndices_) {
		for (auto mc : mc_set) {
			FloatVector sz(getVoxelSize(layer) * 0.5f);
			FloatVector position;
			getPositionFromMortonCode(mc, position, layer);
			out.push_back(position - sz);
			out.push_back(position + sz);
		}
		layer += 1;
	}
}

void Sample_SVOMesh::handleRender()
{
	if (!m_geom || !m_geom->getMesh())
		return;

	Sample_TileMesh::handleRender();

	if (!hasBuild_) return;

	// 1. 画最上层的那个立方体
	/*duDebugDrawBoxWire(
		&m_dd, 
		origin_.x - extent_.x, origin_.y - extent_.y, origin_.z - extent_.z, 
		origin_.x + extent_.x, origin_.y + extent_.y, origin_.z + extent_.z, 
		LINE_COLOR_RED, LINE_WIDTH);*/

	// 2. 画 blocked indices 相关的 box
	std::vector<FloatVector> blockedBox;
	vol_->getAllBlockedBox(blockedBox);

	for (auto i = 0; i < blockedBox.size(); i += 2) {
		auto& minBox = blockedBox[i];
		auto& maxBox = blockedBox[i + 1];
		duDebugDrawBoxWire(
			&m_dd,
			minBox.x, minBox.y, minBox.z,
			maxBox.x, maxBox.y, maxBox.z,
			LINE_COLOR_RED, LINE_WIDTH);
	}
}

bool Sample_SVOMesh::handleBuild()
{
	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();

	float tmp = MAX((bmax[0] - bmin[0]) / 2, MAX((bmax[1] - bmin[1]) / 2, (bmax[2] - bmin[2]) / 2));
	extent_ = FloatVector(1 << ceillog2(ceilf(tmp)));

	origin_ = FloatVector(
		floorf(bmin[0] + (bmax[0] - bmin[0]) / 2),
		floorf(bmin[1] + (bmax[1] - bmin[1]) / 2),
		floorf(bmin[2] + (bmax[2] - bmin[2]) / 2)
	);

	m_ctx->log(RC_LOG_PROGRESS, "origin (%f, %f, %f)", origin_.x, origin_.y, origin_.z);
	m_ctx->log(RC_LOG_PROGRESS, "extent (%f, %f, %f)", extent_.x, extent_.y, extent_.z);

	if (!vol_) {
		vol_ = std::make_shared<SVONVolume>(
			INPUT_VOXEL_POWER, 
			std::bind(&Sample_SVOMesh::getVolumeBoundingBox, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&Sample_SVOMesh::overlapBoxBlockingText, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
		);
	}

	vol_->generate();

	hasBuild_ = true;

	return true;
}

bool Sample_SVOMesh::overlapBoxBlockingText(const FloatVector& pos, float boxRadius, int layers) {
	const auto* mesh = m_geom->getMesh();
	const auto* tris = mesh->getTris();
	const auto* verts = mesh->getVerts();
	const auto* norm = mesh->getNormals();
	for (int i = 0; i < mesh->getTriCount() * 3; i += 3) {
		FloatTri tri;
		tri.normal = FloatVector(norm[i], norm[i+1], norm[i+2]);
		for (int j = 0; j < 3; ++j) {
			const auto* v = &verts[tris[i + j] * 3];
			tri.indexes[j] = FloatVector(v[0], v[1], v[2]);
		}
		if (overlapBoxAndTri(pos, boxRadius, tri)) {
			return true;
		}
	}
	return false;
}

#define DIRSIZE 8
static FloatVector BOX_DIR[DIRSIZE] = {
	{-1, -1, -1},
	{-1, -1,  1},
	{-1,  1,  1},
	{ 1,  1,  1},
	{ 1,  1, -1},
	{ 1, -1, -1},
	{-1,  1, -1},
	{ 1, -1,  1}
};

static const float DELTA = 0.001f;

bool Sample_SVOMesh::overlapBoxAndTri(const FloatVector& pos, float boxRadius, const FloatTri& tri) {
	// 1. 先检查立方体所有的点是否都在三角面片的同一面
	int flag = -1;	// -1=N/A, 1=小于0，2大于0
	int i = 0;
	for (; i < DIRSIZE; ++i) {
		auto v = (pos + BOX_DIR[i] * boxRadius - tri.indexes[0]).cos(tri.normal);
		if (v > DELTA) {
			if (flag == -1) flag = 2;
			else if (flag == 1) break;	// maybe overlap
		}
		else if (v < -DELTA) {
			if (flag == -1) flag = 1;
			else if (flag == 2) break;
		}
	}
	assert(flag != -1);
	if (i == DIRSIZE) return false;
	// 2. 检查三角形三个顶点是否在某一个面的同一边
	// +x
	float t = pos.x + boxRadius;
	if (tri.indexes[0].x - t > DELTA && 
		tri.indexes[1].x - t > DELTA && 
		tri.indexes[2].x - t > DELTA)
		return false;
	// -x
	t = pos.x - boxRadius;
	if (t - tri.indexes[0].x > DELTA &&
		t - tri.indexes[1].x > DELTA &&
		t - tri.indexes[2].x > DELTA)
		return false;
	// +y
	t = pos.y + boxRadius;
	if (tri.indexes[0].y - t > DELTA &&
		tri.indexes[1].y - t > DELTA &&
		tri.indexes[2].y - t > DELTA)
		return false;
	// -y
	t = pos.y - boxRadius;
	if (t - tri.indexes[0].y > DELTA &&
		t - tri.indexes[1].y > DELTA &&
		t - tri.indexes[2].y > DELTA)
		return false;
	// +z
	t = pos.z + boxRadius;
	if (tri.indexes[0].z - t > DELTA &&
		tri.indexes[1].z - t > DELTA &&
		tri.indexes[2].z - t > DELTA)
		return false;
	// -z
	t = pos.z - boxRadius;
	if (t - tri.indexes[0].z > DELTA &&
		t - tri.indexes[1].z > DELTA &&
		t - tri.indexes[2].z > DELTA)
		return false;
	return true;
}