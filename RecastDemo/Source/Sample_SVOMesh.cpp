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
#define LINE_WIDTH 5.0f
#define LINE_COLOR_RED duRGBA(255, 0, 0, 128)

SVONVolume::SVONVolume(int voxelPower, GetVolumeBoudingBoxFunc&& f1, OverlapBoxBlockingTestFunc&& f2)
	: voxelPowner_(voxelPower), boundFunc_(std::move(f1)), checkFunc_(std::move(f2)) {}

bool SVONVolume::generate()
{
	setupVolume();

	return true;
}

void Sample_SVOMesh::handleRender()
{
	if (!m_geom || !m_geom->getMesh())
		return;

	Sample_TileMesh::handleRender();

	if (!hasBuild_) return;

	// 1. 画最上层的那个立方体
	duDebugDrawBoxWire(
		&m_dd, 
		origin_.x - extent_.x, origin_.y - extent_.y, origin_.z - extent_.z, 
		origin_.x + extent_.x, origin_.y + extent_.y, origin_.z + extent_.z, 
		LINE_COLOR_RED, LINE_WIDTH);
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
			3, 
			std::bind(&Sample_SVOMesh::getVolumeBoundingBox, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&Sample_SVOMesh::overlapBoxBlockingText, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
		);
	}

	vol_->generate();

	hasBuild_ = true;

	return true;
}

bool Sample_SVOMesh::overlapBoxBlockingText(const FloatVector& pos, float boxRadius, int layers) {
	return true;
}