#pragma once

#include <random>

#include "Eigen/Dense"
#include "Eigen/StdVector"
#include "Rand.h"

const int gInvalidIdx = -1;

// for convenience define standard vector for rendering
typedef Eigen::Vector4d tVector;
typedef Eigen::Vector4d tVector3;
typedef Eigen::Matrix4d tMatrix;
typedef Eigen::Matrix3d tMatrix3;
typedef std::vector<tVector, Eigen::aligned_allocator<tVector>> tVectorArr;
typedef Eigen::Quaterniond tQuaternion;

const double gRadiansToDegrees = 57.2957795;
const double gDegreesToRadians = 1.0 / gRadiansToDegrees;
const tVector gGravity = tVector(0, -9.8, 0, 0);
const double gInchesToMeters = 0.0254;

class cMathUtil {
  public:
    enum eAxis { eAxisX, eAxisY, eAxisZ, eAxisMax };

    static int Clamp(int val, int min, int max);
    static void Clamp(const Eigen::VectorXd &min, const Eigen::VectorXd &max, Eigen::VectorXd &out_vec);
    static double Clamp(double val, double min, double max);
    static double Saturate(double val);
    static double Lerp(double t, double val0, double val1);

    // rand number
    static double RandDouble();
    static double RandDouble(double min, double max);
    static double RandDoubleNorm(double mean, double stdev);
    static double RandDoubleExp(double lambda);
    static double RandDoubleSeed(double seed);
    static int RandInt();
    static int RandInt(int min, int max);
    static int RandUint();
    static int RandUint(unsigned int min, unsigned int max);
    static int RandIntExclude(int min, int max, int exc);
    static void SeedRand(unsigned long int seed);
    static int RandSign();
    static bool FlipCoin(double p = 0.5);
    static double SmoothStep(double t);

    // matrices
    static tMatrix TranslateMat(const tVector &trans);
    static tMatrix ScaleMat(double scale);
    static tMatrix ScaleMat(const tVector &scale);
    static tMatrix RotateMat(const tVector &euler); // euler angles order rot(Z) * rot(Y) * rot(X)
    static tMatrix RotateMat(const tVector &axis, double theta);
    static tMatrix RotateMat(const tQuaternion &q);
    static tMatrix CrossMat(const tVector &a);
    // inverts a transformation consisting only of rotations and translations
    static tMatrix InvRigidMat(const tMatrix &mat);
    static tVector InvEuler(const tVector &euler);
    static void RotMatToAxisAngle(const tMatrix &mat, tVector &out_axis, double &out_theta);
    static tVector RotMatToEulerAngles(const tMatrix &mat);
    static tQuaternion RotMatToQuaternion(const tMatrix &mat);
    static void EulerToAxisAngle(const tVector &euler, tVector &out_axis, double &out_theta);
    static tMatrix DirToRotMat(const tVector &dir, const tVector &up);

    static void DeltaRot(const tVector &axis0, double theta0, const tVector &axis1, double theta1, tVector &out_axis,
                         double &out_theta);
    static tMatrix DeltaRot(const tMatrix &R0, const tMatrix &R1);

    static tQuaternion EulerToQuaternion(const tVector &euler);
    static tQuaternion AxisAngleToQuaternion(const tVector &axis, double theta);
    static void QuaternionToAxisAngle(const tQuaternion &q, tVector &out_axis, double &out_theta);
    static tMatrix BuildQuaternionDiffMat(const tQuaternion &q);
    static tVector CalcQuaternionVel(const tQuaternion &q0, const tQuaternion &q1, double dt);
    static tVector CalcQuaternionVelRel(const tQuaternion &q0, const tQuaternion &q1, double dt);
    static tQuaternion VecToQuat(const tVector &v);
    static tVector QuatToVec(const tQuaternion &q);
    static tQuaternion QuatDiff(const tQuaternion &q0, const tQuaternion &q1);
    static double QuatDiffTheta(const tQuaternion &q0, const tQuaternion &q1);
    static double QuatTheta(const tQuaternion &dq);
    static tQuaternion VecDiffQuat(const tVector &v0, const tVector &v1);
    static tVector QuatRotVec(const tQuaternion &q, const tVector &dir);
    static tQuaternion MirrorQuaternion(const tQuaternion &q, eAxis axis);

    static double Sign(double val);
    static int Sign(int val);

    static double AddAverage(double avg0, int count0, double avg1, int count1);
    static tVector AddAverage(const tVector &avg0, int count0, const tVector &avg1, int count1);
    static void AddAverage(const Eigen::VectorXd &avg0, int count0, const Eigen::VectorXd &avg1, int count1,
                           Eigen::VectorXd &out_result);
    static void CalcSoftmax(const Eigen::VectorXd &vals, double temp, Eigen::VectorXd &out_prob);
    static double EvalGaussian(const Eigen::VectorXd &mean, const Eigen::VectorXd &covar,
                               const Eigen::VectorXd &sample);
    static double EvalGaussian(double mean, double covar, double sample);
    static double CalcGaussianPartition(const Eigen::VectorXd &covar);
    static double EvalGaussianLogp(double mean, double covar, double sample);
    static double EvalGaussianLogp(const Eigen::VectorXd &mean, const Eigen::VectorXd &covar,
                                   const Eigen::VectorXd &sample);
    static double Sigmoid(double x);
    static double Sigmoid(double x, double gamma, double bias);

    static int SampleDiscreteProb(const Eigen::VectorXd &probs);
    static tVector CalcBarycentric(const tVector &p, const tVector &a, const tVector &b, const tVector &c);

    static bool ContainsAABB(const tVector &pt, const tVector &aabb_min, const tVector &aabb_max);
    static bool ContainsAABB(const tVector &aabb_min0, const tVector &aabb_max0, const tVector &aabb_min1,
                             const tVector &aabb_max1);
    static bool ContainsAABBXZ(const tVector &pt, const tVector &aabb_min, const tVector &aabb_max);
    static bool ContainsAABBXZ(const tVector &aabb_min0, const tVector &aabb_max0, const tVector &aabb_min1,
                               const tVector &aabb_max1);
    static void CalcAABBIntersection(const tVector &aabb_min0, const tVector &aabb_max0, const tVector &aabb_min1,
                                     const tVector &aabb_max1, tVector &out_min, tVector &out_max);
    static void CalcAABBUnion(const tVector &aabb_min0, const tVector &aabb_max0, const tVector &aabb_min1,
                              const tVector &aabb_max1, tVector &out_min, tVector &out_max);
    static bool IntersectAABB(const tVector &aabb_min0, const tVector &aabb_max0, const tVector &aabb_min1,
                              const tVector &aabb_max1);
    static bool IntersectAABBXZ(const tVector &aabb_min0, const tVector &aabb_max0, const tVector &aabb_min1,
                                const tVector &aabb_max1);

  private:
    static cRand gRand;

    template <typename T> static T SignAux(T val) { return (T(0) < val) - (val < T(0)); }
};
