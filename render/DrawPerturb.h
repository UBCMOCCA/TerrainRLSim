#pragma once

#include "sim/Perturb.h"

class cDrawPerturb {
  public:
    static const double gForceScale;
    static const double gTorqueScale;

    static void DrawForce2D(const tVector &pos, const tVector &force);
    static void DrawTorque2D(const tVector &pos, const tVector &torque);

    static void Draw(const tPerturb &perturb);

  protected:
    static void DrawForce(const tPerturb &perturb);
    static void DrawTorque(const tPerturb &perturb);
};
