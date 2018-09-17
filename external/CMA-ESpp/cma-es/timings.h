#pragma once

#include "utils.h"
#include <ctime>
#include <cassert>

/**
 * @class Timing 
 * A class for time measurements of the eigen decomposition.
 * Timing measures overall time and times between calls of tic and toc. For
 * small time spans (up to 1000 seconds) CPU time via clock() is used. For large
 * time spans the fall-back to elapsed time from time() is used.
 * timings_update() must be called often enough to prevent the fallback.
 */
class Timing
{
  clock_t lastclock;
  time_t lasttime;
  clock_t ticclock;
  time_t tictime;
  short istic;
  short isstarted;

  double lastdiff;
  double tictoczwischensumme;

public:
  double totaltime; //! zeroed by re-calling timings_start
  double totaltotaltime;
  double tictoctime;
  double lasttictoctime;

  Timing()
  {
    totaltotaltime = 0;
    start();
  }

  void start()
  {
    totaltime = 0;
    tictoctime = 0;
    lasttictoctime = 0;
    istic = 0;
    lastclock = clock();
    lasttime = time(NULL);
    lastdiff = 0;
    tictoczwischensumme = 0;
    isstarted = 1;
  }

  /**
   * @return time between last call of timings_*() and now
   */
  double update()
  {
    double diffc, difft;
    clock_t lc = lastclock; // measure CPU in 1e-6s
    time_t lt = lasttime; // measure time in s

    assert(isstarted == 1 && "timings_started() must be called before using timings... functions");

    lastclock = clock(); // measures at most 2147 seconds, where 1s = 1e6 CLOCKS_PER_SEC
    lasttime = time(NULL);
    diffc = (double) (lastclock - lc) / CLOCKS_PER_SEC; // is presumably in [-21??, 21??]
    difft = difftime(lasttime, lt); // is presumably an integer
    lastdiff = difft; // on the "save" side
    // use diffc clock measurement if appropriate
    if(diffc > 0 && difft < 1000)
      lastdiff = diffc;
    assert(lastdiff >= 0 && "BUG in time measurement");
    totaltime += lastdiff;
    totaltotaltime += lastdiff;
    if(istic)
    {
      tictoczwischensumme += lastdiff;
      tictoctime += lastdiff;
    }
    return lastdiff;
  }

  void tic()
  {
    assert(!istic && "Timingic called twice without toc");
    update();
    istic = 1;
  }

  double toc()
  {
    assert(istic && "Timingoc called without tic");
    update();
    lasttictoctime = tictoczwischensumme;
    tictoczwischensumme = 0;
    istic = 0;
    return lasttictoctime;
  }
};
