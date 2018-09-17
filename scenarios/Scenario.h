#pragma once

#include "util/ArgParser.h"
#include <functional>
#include <memory>
#include <string>

class cScenario {
  public:
    typedef std::function<void()> tCallbackFunc;
    typedef std::function<void(double)> tTimeCallbackFunc;

    virtual ~cScenario();

    virtual void Init();
    virtual void ParseArgs(const std::shared_ptr<cArgParser> &parser);
    virtual void Reset();
    virtual void Clear();
    virtual void Run();
    virtual void Shutdown();

    virtual bool IsDone() const;
    virtual void Update(double time_elapsed);
    virtual void SetResetCallback(tCallbackFunc func);

    virtual std::string GetName() const;
    virtual void setRelativeFilePath(std::string path);
    virtual std::string getRelativeFilePath();

  protected:
    tCallbackFunc mResetCallback;
    std::string _relativeFilePath;

    cScenario();
};
