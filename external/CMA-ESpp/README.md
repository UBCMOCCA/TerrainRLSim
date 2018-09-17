# SOURCE

The code is based on Hansen's ANSI C implementation of CMA-ES:

  http://www.lri.fr/~hansen/cmaes_inmatlab.html#C

# HOW TO START

1. You need the following programs:
    * cmake - in order to generate make files (it is only tested with these)
    * doxygen - if you want to read the API documentation you could read the
                comments in each file, but I would rather suggest reading the
                generated html documentation
    * g++ - or any other C++ compiler
2. Take five minutes to look at file example1.cpp.
3. Compile and run the example programs. Compilation e.g. with the GNU
   c++-compiler and run with "./evo1" or "./evo2". Take a look at the output.
   * mkdir build
   * cd build
   * cmake ..
   * make
4. (optional) Change (increase) problem dimension and/or problem
   number in the example code.
5. Read the generated API documentation!
   (http://alexanderfabisch.github.com/CMA-ESpp/html/)
6. Now you are ready to inspect and edit example2.cpp.
7. Make sure that the scale of all objective parameter components of the
   function is somewhat similar and sigma corresponds to about 1/4 of the
   respective search intervals.

