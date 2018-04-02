#include "ScenarioKinMotion.h"

cScenarioKinMotion::cScenarioKinMotion()
{
	mCharInitPos.setZero();
}

cScenarioKinMotion::~cScenarioKinMotion()
{
}

void cScenarioKinMotion::Init()
{
	bool succ = BuildCharacter(mCharFile, mMotionFile);
	if (!succ)
	{
		printf("Failed to build character from char_file: %s, motion_file: %s\n", mCharFile.c_str(), mMotionFile.c_str());
	}
}

void cScenarioKinMotion::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	bool succ = parser->ParseString("character_file", mCharFile);
	if (!succ)
	{
		printf("No character file provided\n");
	}
	else{
		std::string fpath;
		succ = parser->ParseString("relative_file_path", fpath);
		if (succ)
		{
			mCharFile = fpath + mCharFile;
		}
	}

	parser->ParseString("motion_file", mMotionFile);
	std::string fpath;
	succ = parser->ParseString("relative_file_path", fpath);
	if (succ)
	{
		mMotionFile = fpath + mMotionFile;
	}

	parser->ParseDouble("char_init_pos_x", mCharInitPos[0]);
}

void cScenarioKinMotion::Reset()
{
	mChar.Reset();
}

void cScenarioKinMotion::Clear()
{
	mChar.Clear();
}

void cScenarioKinMotion::Update(double time_elapsed)
{
	mChar.Update(time_elapsed);
}

const cKinCharacter& cScenarioKinMotion::GetCharacter() const
{
	return mChar;
}

tVector cScenarioKinMotion::GetCharPos() const
{
	return GetCharacter().GetRootPos();
}

double cScenarioKinMotion::GetTime() const
{
	return mChar.GetTime();
}

std::string cScenarioKinMotion::GetName() const
{
	return "Kinematic Motion";
}

bool cScenarioKinMotion::BuildCharacter(const std::string& char_file, const std::string& motion_file)
{
	bool succ = mChar.Init(char_file, motion_file);

	if (succ)
	{
		tVector origin = mChar.GetOriginPos();
		origin[0] = mCharInitPos[0];
		mChar.SetOriginPos(origin);
	}
	
	return succ;
}

