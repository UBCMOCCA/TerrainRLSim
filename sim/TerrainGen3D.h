#pragma once

#include "Ground.h"
#include "util/ArgParser.h"
#include "util/JsonUtil.h"
#include "util/MathUtil.h"
#include "util/Rand.h"
#include <vector>

class cTerrainGen3D {
  public:
    enum eVertFlag { eVertFlagEnableTex, eVertFlagMax };

    enum eParams {
        eParamsPathWidth0,
        eParamsPathWidth1,
        eParamsPathWidthDeltaStdev,
        eParamsPathMaxTurnDelta,
        eParamsPathLength,
        eParamsPathTurnLength,
        eParamsPathHeight0,
        eParamsPathHeight1,
        eParamsStepHeightMin,
        eParamsStepHeightMax,
        eParamsTrailStepProb,
        eParamsNumObstacles,
        eParamsObstacleDensity,
        eParamsObstacleWidth0,
        eParamsObstacleWidth1,
        eParamsObstacleHeight0,
        eParamsObstacleHeight1,
        eParamsObstacleSpeed0,
        eParamsObstacleSpeed1,
        eParamsObstacleSpeedLerpPow,
        eParamsSlope,
        eParamsConveyorStripLength,
        eParamsConveyorNumStrips,
        eParamsConveyorNumSlices,
        eParamsConveyorSpacing,
        eParamsStepSpacingMin,
        eParamsStepSpacingMax,
        eParamsMax
    };

    struct tParamDef {
        std::string mName;
        double mDefaultVal;
    };

    static const double gInvalidHeight;
    static const tParamDef gParamDefs[];
    static void GetDefaultParams(Eigen::VectorXd &out_params);
    static void LoadParams(const Json::Value &root, Eigen::VectorXd &out_params);
    static int CalcResX(double x_size, double vert_spacing);
    static int CalcResZ(double z_size, double vert_spacing);
    static int CalcNumVerts(const tVector &ground_size, double spacing_x, double spacing_z);

    static const float gVertSpacing;

    typedef void (*tTerrainFunc)(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                                 const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                                 std::vector<int> &out_flags);
    static tTerrainFunc GetTerrainFunc(cGround::eType terrain_type);

    static void BuildFlat(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                          const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                          std::vector<int> &out_flags);
    static void BuildPath(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                          const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                          std::vector<int> &out_flags);
    static void BuildCliff(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                           const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                           std::vector<int> &out_flags);
    static void BuildRamp(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                          const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                          std::vector<int> &out_flags);

    static void BuildCheckers(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                              const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                              std::vector<int> &out_flags);

    static void BuildStairs(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                            const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                            std::vector<int> &out_flags);

  protected:
    static void AddFlat(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                        double spacing_x, double spacing_z, const Eigen::Vector2i &out_res,
                        std::vector<float> &out_data, std::vector<int> &out_flags);
    static void AddPath(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                        double spacing_x, double spacing_z, const Eigen::Vector2i &out_res, double w, double len,
                        double turn_len, double h0, double h1, std::vector<float> &out_data,
                        std::vector<int> &out_flags);
    static void AddCliff(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                         double spacing_x, double spacing_z, const Eigen::Vector2i &out_res,
                         std::vector<float> &out_data, std::vector<int> &out_flags);
    static void AddRamp(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                        double spacing_x, double spacing_z, double slope, const Eigen::Vector2i &out_res,
                        std::vector<float> &out_data, std::vector<int> &out_flags);

    static void AddCheckers(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                            double spacing_x, double spacing_z, double slope, const Eigen::Vector2i &out_res,
                            std::vector<float> &out_data, std::vector<int> &out_flags);

    static void AddStairs(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                          double spacing_x, double spacing_z, double spacing_min, double spacing_max, double step_h_min,
                          double step_h_max, const Eigen::Vector2i &out_res, cRand &rand, std::vector<float> &out_data,
                          std::vector<int> &out_flags);
};
