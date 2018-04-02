
 /* simAdapter.i */
 %module terrainRLAdapter
 %{
 /* Put header files here or function declarations like below */
 #define SWIG_FILE_WITH_INIT
 #include <vector>

 
#include "SimAdapter.h"
// #include "../sim/TerrainRLCharController.h"
// #include "../sim/SimCharacter.h"
// #include "../util/MathUtil.h"
 
%}
%include "std_string.i"
%include "std_vector.i"
 // %include "std_pair.i"
 // Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(ConstCharVector) vector<const char*>;
}

%rename(at) operator[];
%rename(add) operator+;
%rename(multiply) operator*;
%rename(divide) operator/;
%rename(subtract) operator-;
%rename(lessThan) operator<;
%rename(equal) operator==;
%rename(notEqual) operator!=;
%rename(incrementAdd) operator+=;
%rename(incrementSubtract) operator-=;
%rename(incrementMultiply) operator*=;
%rename(incrementDivide) operator/=;

%include "SimAdapter.h"
// %include "../sim/TerrainRLCharController.h"
// %include "../sim/SimCharacter.h"
// %include "../util/MathUtil.h"

 