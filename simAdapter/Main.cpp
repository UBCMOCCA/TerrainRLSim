
#include <SimAdapter.h>
// #include "render/DrawUtil.h"

int main(int argc, char** argv)
{

	std::vector<std::string> args;
	for (size_t i=0; i < argc; i++)
	{
		args.push_back(std::string(argv[i]));
	}
	cSimAdapter sim(args);
	sim.init();


	return 0;
}

