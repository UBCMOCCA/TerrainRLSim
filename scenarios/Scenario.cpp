#include "Scenario.h"

cScenario::cScenario()
{
	mResetCallback = nullptr;
	_relativeFilePath = "";
}

void cScenario::Init()
{
}

void cScenario::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{

}

void cScenario::Reset()
{
	if (mResetCallback != nullptr)
	{
		mResetCallback();
	}
}

void cScenario::Clear()
{
}

void cScenario::Run()
{
}

void cScenario::Shutdown()
{
}

bool cScenario::IsDone() const
{
	return false;
}

void cScenario::Update(double time_elapsed)
{
}

void cScenario::SetResetCallback(tCallbackFunc func)
{
	mResetCallback = func;
}

std::string cScenario::GetName() const
{
	return "No Name";
}

cScenario::~cScenario()
{
}

void cScenario::setRelativeFilePath(std::string path)
{
	_relativeFilePath = path;
}

std::string cScenario::getRelativeFilePath()
{
	return _relativeFilePath;
}
