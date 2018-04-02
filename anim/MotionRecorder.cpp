#include "MotionRecorder.h"
#include <assert.h>
#include <iostream>

#include "util/FileUtil.h"

cMotionRecorder::cMotionRecorder()
{
}

cMotionRecorder::~cMotionRecorder()
{
}

void cMotionRecorder::Reset()
{
	mFrameBuffer.clear();
}

int cMotionRecorder::GetNumFrames() const
{
	return static_cast<int>(mFrameBuffer.size());
}

int cMotionRecorder::GetFrameSize() const
{
	int frame_size = gInvalidIdx;
	if (GetNumFrames() > 0)
	{
		frame_size = static_cast<int>(mFrameBuffer[0].size() - 1); // first entry is duration of frame
	}
	return frame_size;
}

bool cMotionRecorder::AddFrame(double dur, const Eigen::VectorXd& frame)
{
	int frame_size = GetFrameSize();
	int num_frames = GetNumFrames();
	int in_frame_size = static_cast<int>(frame.size());
	bool valid_frame_size = (in_frame_size == frame_size);
	if (num_frames == 0)
	{
		valid_frame_size = true;
		frame_size = in_frame_size;
	}
	assert(valid_frame_size);

	bool succ = false;
	if (valid_frame_size)
	{
		Eigen::VectorXd frame_entry = Eigen::VectorXd(frame_size + 1);
		frame_entry(cMotion::eFrameTime) = dur;
		frame_entry.segment(cMotion::eFrameMax, frame_size) = frame;
		mFrameBuffer.push_back(frame_entry);
		succ = true;
	}
	return succ;
}

bool cMotionRecorder::OutputMotion(const std::string& out_file) const
{
	bool succ = true;
	FILE* f = cFileUtil::OpenFile(out_file, "w");
	fprintf(f, "{\n\t\"Loop\": true,\n\t\"%s\":\n\t[\n", cMotion::gFrameKey.c_str());
	
	int num_frames = GetNumFrames();
	for (int i = 0; i < num_frames; ++i)
	{
		if (i != 0)
		{
			fprintf(f, ",\n");
		}
		const auto& curr_frame = mFrameBuffer[i];
		std::string json = cMotion::BuildFrameJson(curr_frame);
		fprintf(f, "\t\t%s", json.c_str());
	}

	fprintf(f, "\n\t]\n}");
	cFileUtil::CloseFile(f);

	return succ;
}

std::string cMotionRecorder::BuildFrameJson(const Eigen::VectorXd& frame) const
{
	std::string json = "";
	return json;
}
