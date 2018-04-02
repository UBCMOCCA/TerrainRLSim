#pragma once

#include "Motion.h"
#include "util/MathUtil.h"

class cMotionRecorder
{
public:
	cMotionRecorder();
	virtual ~cMotionRecorder();

	virtual void Reset();
	virtual int GetNumFrames() const;
	virtual int GetFrameSize() const;
	virtual bool AddFrame(double dur, const Eigen::VectorXd& frame);
	virtual bool OutputMotion(const std::string& out_file) const;

protected:
	std::vector<Eigen::VectorXd> mFrameBuffer;

	virtual std::string BuildFrameJson(const Eigen::VectorXd& frame) const;
};
