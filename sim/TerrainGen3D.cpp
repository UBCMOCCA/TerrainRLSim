#include "TerrainGen3D.h"
#include <algorithm>

const double cTerrainGen3D::gInvalidHeight = -std::numeric_limits<double>::infinity();
const float cTerrainGen3D::gVertSpacing = 0.2f;

const cTerrainGen3D::tParamDef cTerrainGen3D::gParamDefs[] = {
    {"PathWidth0", 3},           {"PathWidth1", 3},           {"PathWidthDeltaStdev", 0.1},
    {"PathMaxTurnDelta", 1.5},   {"PathLength", 10},          {"PathTurnLength", 5},
    {"PathHeight0", 0},          {"PathHeight1", -2},         {"StepHeightMin", 0.1},
    {"StepHeightMax", 0.2},      {"TrailStepProb", 0.2},      {"NumObstacles", 10},
    {"ObstacleDensity", 0.015},  {"ObstacleWidth0", 0.5},     {"ObstacleWidth1", 7},
    {"ObstacleHeight0", 1},      {"ObstacleHeight1", 5},      {"ObstacleSpeed0", 0.1},
    {"ObstacleSpeed1", 2},       {"ObstacleSpeedLerpPow", 1}, {"Slope", 0},
    {"ConveyorStripLength", 10}, {"ConveyorNumStrips", 4},    {"ConveyorNumSlices", 10},
    {"ConveyorSpacing", 3},

    {"StepSpacingMin", 1},       {"StepSpacingMax", 2}};

void cTerrainGen3D::GetDefaultParams(Eigen::VectorXd &out_params) {
    out_params = Eigen::VectorXd::Zero(eParamsMax);
    assert(sizeof(gParamDefs) / sizeof(gParamDefs[0]) == eParamsMax);
    for (int i = 0; i < eParamsMax; ++i) {
        out_params[i] = gParamDefs[i].mDefaultVal;
    }
}

void cTerrainGen3D::LoadParams(const Json::Value &root, Eigen::VectorXd &out_params) {
    cTerrainGen3D::GetDefaultParams(out_params);
    for (int i = 0; i < eParamsMax; ++i) {
        const std::string &name = gParamDefs[i].mName;
        if (!root[name].isNull()) {
            double val = root[name].asDouble();
            out_params[i] = val;
        }
    }
}

int cTerrainGen3D::CalcNumVerts(const tVector &ground_size, double spacing_x, double spacing_z) {
    int res_x = CalcResX(ground_size[0], spacing_x);
    int res_z = CalcResZ(ground_size[2], spacing_z);
    return res_x * res_z;
}

int cTerrainGen3D::CalcResX(double x_size, double vert_spacing) {
    return static_cast<int>(std::ceil(x_size / vert_spacing - 0.000001)) + 1;
}

int cTerrainGen3D::CalcResZ(double z_size, double vert_spacing) {
    return static_cast<int>(std::ceil(z_size / vert_spacing - 0.000001)) + 1;
}

cTerrainGen3D::tTerrainFunc cTerrainGen3D::GetTerrainFunc(cGround::eType terrain_type) {
    switch (terrain_type) {
    case cGround::eTypeVar3DFlat:
        return BuildFlat;
    case cGround::eTypeVar3DPath:
        return BuildPath;
    case cGround::eTypeVar3DCliff:
        return BuildCliff;
    case cGround::eTypeVar3DRamp:
        return BuildRamp;
    case cGround::eTypeVar3DCheckers:
        return BuildCheckers;
    case cGround::eTypeVar3DStairs:
        return BuildStairs;
    default:
        printf("Unsupported ground var3d type.\n");
        assert(false);
        return BuildFlat;
    }
}

void cTerrainGen3D::BuildFlat(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                              const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                              std::vector<int> &out_flags) {
    Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
    Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
    out_res[0] = CalcResX(ground_size[0], spacing_x);
    out_res[1] = CalcResZ(ground_size[2], spacing_z);
    return AddFlat(origin, start_coord, ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cTerrainGen3D::BuildPath(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                              const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                              std::vector<int> &out_flags) {
    double w = params[eParamsPathWidth0];
    double len = params[eParamsPathLength];
    double turn_len = params[eParamsPathTurnLength];
    double h0 = params[eParamsPathHeight0];
    double h1 = params[eParamsPathHeight1];

    Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
    Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
    out_res[0] = CalcResX(ground_size[0], spacing_x);
    out_res[1] = CalcResZ(ground_size[2], spacing_z);
    return AddPath(origin, start_coord, ground_size, spacing_x, spacing_z, out_res, w, len, turn_len, h0, h1, out_data,
                   out_flags);
}

void cTerrainGen3D::BuildCliff(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                               const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                               std::vector<int> &out_flags) {
    Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
    Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
    out_res[0] = CalcResX(ground_size[0], spacing_x);
    out_res[1] = CalcResZ(ground_size[2], spacing_z);
    return AddCliff(origin, start_coord, ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cTerrainGen3D::BuildRamp(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                              const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                              std::vector<int> &out_flags) {
    double slope = params[eParamsSlope];

    Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
    Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
    out_res[0] = CalcResX(ground_size[0], spacing_x);
    out_res[1] = CalcResZ(ground_size[2], spacing_z);
    return AddRamp(origin, start_coord, ground_size, spacing_x, spacing_z, slope, out_res, out_data, out_flags);
}

void cTerrainGen3D::BuildCheckers(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                                  const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                                  std::vector<int> &out_flags) {
    double slope = params[eParamsSlope];

    Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
    Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
    out_res[0] = CalcResX(ground_size[0], spacing_x);
    out_res[1] = CalcResZ(ground_size[2], spacing_z);
    return AddCheckers(origin, start_coord, ground_size, spacing_x, spacing_z, slope, out_res, out_data, out_flags);
}

void cTerrainGen3D::BuildStairs(const tVector &origin, const tVector &ground_size, double spacing_x, double spacing_z,
                                const Eigen::VectorXd &params, cRand &rand, std::vector<float> &out_data,
                                std::vector<int> &out_flags) {
    double spacing_min = params[eParamsStepSpacingMin];
    double spacing_max = params[eParamsStepSpacingMax];
    double step_h_min = params[eParamsStepHeightMin];
    double step_h_max = params[eParamsStepHeightMax];

    Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
    Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
    out_res[0] = CalcResX(ground_size[0], spacing_x);
    out_res[1] = CalcResZ(ground_size[2], spacing_z);
    return AddStairs(origin, start_coord, ground_size, spacing_x, spacing_z, spacing_min, spacing_max, step_h_min,
                     step_h_max, out_res, rand, out_data, out_flags);
}

void cTerrainGen3D::AddFlat(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                            double spacing_x, double spacing_z, const Eigen::Vector2i &out_res,
                            std::vector<float> &out_data, std::vector<int> &out_flags) {
    int res_x = CalcResX(size[0], spacing_x);
    int res_z = CalcResX(size[2], spacing_z);
    int num_verts = res_x * res_z;

    for (int j = 0; j < res_z; ++j) {
        size_t coord_z = j + start_coord[1];
        for (int i = 0; i < res_x; ++i) {
            size_t coord_x = i + start_coord[0];
            size_t idx = coord_z * out_res[0] + coord_x;
            double h = 0;
            int curr_flags = 1 << eVertFlagEnableTex;

            out_data[idx] = static_cast<float>(h);
            out_flags[idx] = curr_flags;
        }
    }
}

void cTerrainGen3D::AddPath(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                            double spacing_x, double spacing_z, const Eigen::Vector2i &out_res, double w, double len,
                            double turn_len, double h0, double h1, std::vector<float> &out_data,
                            std::vector<int> &out_flags) {
    int res_x = CalcResX(size[0], spacing_x);
    int res_z = CalcResX(size[2], spacing_z);
    int num_verts = res_x * res_z;

    for (int j = 0; j < res_z; ++j) {
        int coord_z = j + start_coord[1];
        for (int i = 0; i < res_x; ++i) {
            int coord_x = i + start_coord[0];
            int idx = coord_z * out_res[0] + coord_x;
            tVector curr_pos = origin + tVector(i * spacing_x, 0, j * spacing_z, 0);

            double curr_h = h1;
            int interval = static_cast<int>(std::floor(curr_pos[0] / len));
            if (interval % 2 == 0) {
                curr_h = (std::abs(curr_pos[2]) < 0.5 * w) ? h0 : h1;
            } else {
                curr_h = (std::abs(turn_len - curr_pos[2]) < 0.5 * w) ? h0 : h1;
            }

            double interval_start = interval * len;
            double interval_end = (interval + 1) * len;
            if (std::abs(interval_start - curr_pos[0]) < 0.5 * w || std::abs(interval_end - curr_pos[0]) < 0.5 * w) {
                if (curr_pos[2] > -0.5 * w && curr_pos[2] < turn_len + 0.5 * w) {
                    curr_h = h0;
                }
            }

            out_data[idx] = static_cast<float>(curr_h);
        }
    }

    for (int j = 0; j < res_z; ++j) {
        int coord_z = j + start_coord[1];
        for (int i = 0; i < res_x; ++i) {
            int coord_x = i + start_coord[0];
            int idx0 = coord_z * out_res[0] + coord_x;
            int idx1 = cMathUtil::Clamp(coord_z - 1, 0, res_z - 1) * out_res[0] + coord_x;
            int idx2 = cMathUtil::Clamp(coord_z + 1, 0, res_z - 1) * out_res[0] + coord_x;
            int idx3 = coord_z * out_res[0] + cMathUtil::Clamp(coord_x - 1, 0, res_x - 1);
            int idx4 = coord_z * out_res[0] + cMathUtil::Clamp(coord_x + 1, 0, res_x - 1);

            double mid = out_data[idx0];
            double n = out_data[idx1];
            double s = out_data[idx2];
            double w = out_data[idx3];
            double e = out_data[idx4];

            bool on_edge = (mid == h1) && ((n == h0) || (s == h0) || (w == h0) || (e == h0));
            int curr_flags = 0;
            if (!on_edge) {
                curr_flags = 1 << eVertFlagEnableTex;
            }
            out_flags[idx0] = curr_flags;
        }
    }
}

void cTerrainGen3D::AddCliff(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                             double spacing_x, double spacing_z, const Eigen::Vector2i &out_res,
                             std::vector<float> &out_data, std::vector<int> &out_flags) {
    const double h0 = 0;
    const double h1 = 2;
    const double h2 = -2;
    const double w = 5;
    const double period_len = 12;
    const double amplitude = 5;
    const double slope = 0.5;
    const double bump_size = 0.2;

    int res_x = CalcResX(size[0], spacing_x);
    int res_z = CalcResX(size[2], spacing_z);
    int num_verts = res_x * res_z;

    for (int j = 0; j < res_z; ++j) {
        size_t coord_z = j + start_coord[1];
        for (int i = 0; i < res_x; ++i) {
            size_t coord_x = i + start_coord[0];
            size_t idx = coord_z * out_res[0] + coord_x;
            tVector curr_pos = origin + tVector(i * spacing_x, 0, j * spacing_z, 0);
            int curr_flags = 0;

            double curr_h = h0;
            double mode_val = std::cos(2 * M_PI * curr_pos[0] / period_len) - 1;
            // double mode_val = 0.5 * std::sin(2 * M_PI * curr_pos[0] / period_len)
            //				+ 2 * std::cos(0.5 * M_PI * curr_pos[0] / period_len);
            // mode_val *= 0.5 + std::cos(0.1 * M_PI * curr_pos[0] / period_len);

            mode_val *= amplitude;
            double curr_dist = curr_pos[2] - mode_val;

            if (curr_dist < -0.5 * w) {
                curr_h = h1;
            } else if (curr_dist > 0.5 * w) {
                curr_h = h2;
            }

            double h_lerp = std::abs(curr_dist) - 0.5 * w;
            h_lerp *= slope;
            h_lerp = cMathUtil::Clamp(h_lerp, 0.0, 1.0);
            curr_h = (1 - h_lerp) * h0 + h_lerp * curr_h;

            bool on_edge = (h_lerp > 0) && (h_lerp < 1);
            if (on_edge) {
                double bump_h = cMathUtil::RandDoubleSeed(curr_pos[0] * curr_pos[0] * M_PI + curr_pos[2]);
                bump_h = 2 * bump_h - 1;
                bump_h *= bump_size;
                curr_h += bump_h;
            } else {
                curr_flags |= 1 << eVertFlagEnableTex;
            }

            out_data[idx] = static_cast<float>(curr_h);
            out_flags[idx] = curr_flags;
        }
    }
}

void cTerrainGen3D::AddRamp(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                            double spacing_x, double spacing_z, double slope, const Eigen::Vector2i &out_res,
                            std::vector<float> &out_data, std::vector<int> &out_flags) {
    const double pad = 1;

    int res_x = CalcResX(size[0], spacing_x);
    int res_z = CalcResX(size[2], spacing_z);
    int num_verts = res_x * res_z;

    for (int j = 0; j < res_z; ++j) {
        size_t coord_z = j + start_coord[1];
        for (int i = 0; i < res_x; ++i) {
            size_t coord_x = i + start_coord[0];
            size_t idx = coord_z * out_res[0] + coord_x;
            double h = 0;
            int curr_flags = 1 << eVertFlagEnableTex;

            tVector pos = origin;
            pos[0] += (i / (res_x - 1.0)) * size[0];
            pos[2] += (j / (res_z - 1.0)) * size[2];

            double x = pos[0];
            x = cMathUtil::Sign(x) * std::max(0.0, (std::abs(x) - pad));
            h = slope * x;

            out_data[idx] = static_cast<float>(h);
            out_flags[idx] = curr_flags;
        }
    }
}

void cTerrainGen3D::AddCheckers(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                                double spacing_x, double spacing_z, double slope, const Eigen::Vector2i &out_res,
                                std::vector<float> &out_data, std::vector<int> &out_flags) {
    const double pad = 1;

    int res_x = CalcResX(size[0], spacing_x);
    int res_z = CalcResX(size[2], spacing_z);
    int num_verts = res_x * res_z;

    for (int j = 0; j < res_z; ++j) {
        size_t coord_z = j + start_coord[1];
        for (int i = 0; i < res_x; ++i) {
            size_t coord_x = i + start_coord[0];
            size_t idx = coord_z * out_res[0] + coord_x;
            double h = 0;
            int curr_flags = 1 << eVertFlagEnableTex;

            tVector pos = origin;
            pos[0] += (i / (res_x - 1.0)) * size[0];
            pos[2] += (j / (res_z - 1.0)) * size[2];

            double x = pos[0];
            double z = pos[2];
            x = std::max(0.0, (std::abs(x)));
            z = std::max(0.0, (std::abs(z)));
            h = ((int)std::floor(z) + (int)std::floor(x)) % 2 * slope;

            out_data[idx] = h;
            out_flags[idx] = curr_flags;
        }
    }
}

void cTerrainGen3D::AddStairs(const tVector &origin, const Eigen::Vector2i &start_coord, const tVector &size,
                              double spacing_x, double spacing_z, double spacing_min, double spacing_max,
                              double step_h_min, double step_h_max, const Eigen::Vector2i &out_res, cRand &rand,
                              std::vector<float> &out_data, std::vector<int> &out_flags) {
    const double pad = 1;

    int res_x = CalcResX(size[0], spacing_x);
    int res_z = CalcResX(size[2], spacing_z);
    int num_verts = res_x * res_z;
    // double last_step_z = origin[2];

    // double current_spacing = rand.RandDouble(spacing_min, spacing_max);
    // double current_height = 0;

    double randomness = rand.RandDouble();

    for (int j = 0; j < res_z; ++j) {
        // Check the new step and reset the height if we are done with the last step

        double z = origin[2] + (j / (res_z - 1.0)) * size[2];
        /*
        if (z > (last_step_z + current_spacing))
        {
            last_step_z = z;
            current_height += cMathUtil::Sign(z)*rand.RandDouble(step_h_min, step_h_max);
            current_spacing = rand.RandDouble(spacing_min, spacing_max);
        }
        */

        size_t coord_z = j + start_coord[1];
        for (int i = 0; i < res_x; ++i) {
            size_t coord_x = i + start_coord[0];
            size_t idx = coord_z * out_res[0] + coord_x;

            double x = origin[0] + (i / (res_x - 1.0)) * size[0];

            x = cMathUtil::Sign(x) * std::max(0.0, (std::abs(x) - pad));

            double h = (std::floor(x * 2)) * 0.1 / 2;
            // rand.Seed(h);
            // h += rand.RandDouble(step_h_min, step_h_max);
            h += cMathUtil::RandDoubleSeed(h * randomness) * (step_h_max - step_h_min) + step_h_min;

            int curr_flags = 1 << eVertFlagEnableTex;
            out_data[idx] = h;
            out_flags[idx] = curr_flags;
        }
    }
}
