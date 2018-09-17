#pragma once

#include "anim/Character.h"
#include "sim/CharController.h"
#include "sim/Joint.h"
#include "sim/SimObj.h"
#include "sim/World.h"

class cSimCharacter : public cCharacter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct tParams {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        tParams();

        std::string mCharFile;
        std::string mStateFile;
        std::string mRelativeFilePath;
        tVector mInitPos;

        bool mEnableSoftContact;
        bool mEnableFallDist;
    };

    cSimCharacter();
    virtual ~cSimCharacter();

    virtual bool Init(std::shared_ptr<cWorld> world, const tParams &params);
    virtual void Clear();
    virtual void Reset();
    virtual void Update(double time_step);

    virtual tVector GetRootPos() const;
    virtual void GetRootRotation(tVector &out_axis, double &out_theta) const;
    virtual tQuaternion GetRootRotation() const;
    virtual tVector GetRootVel() const;
    virtual tVector GetRootAngVel() const;
    virtual const Eigen::MatrixXd &GetBodyDefs() const;
    virtual const Eigen::MatrixXd &GetDrawShapeDefs() const;
    virtual void SetRootPos(const tVector &pos);
    virtual void SetRootRotation(const tQuaternion &q);
    virtual void SetRootTransform(const tVector &pos, const tQuaternion &rot);

    virtual tQuaternion CalcHeadingRot() const;

    virtual int GetNumBodyParts() const;

    virtual void SetPose(const Eigen::VectorXd &pose);
    virtual void SetVel(const Eigen::VectorXd &vel);

    virtual tVector CalcJointPos(int joint_id) const;
    virtual tVector CalcJointVel(int joint_id) const;
    virtual void CalcJointWorldRotation(int joint_id, tVector &out_axis, double &out_theta) const;
    virtual tQuaternion CalcJointWorldRotation(int joint_id) const;
    virtual tMatrix BuildJointWorldTrans(int joint_id) const;

    virtual tVector CalcCOM() const;
    virtual tVector CalcCOMVel() const;
    virtual void CalcAABB(tVector &out_min, tVector &out_max) const;

    virtual const cJoint &GetJoint(int joint_id) const;
    virtual cJoint &GetJoint(int joint_id);
    virtual const std::shared_ptr<cSimObj> &GetBodyPart(int idx) const;
    virtual std::shared_ptr<cSimObj> &GetBodyPart(int idx);
    virtual tVector GetBodyPartPos(int idx) const;
    virtual tVector GetBodyPartVel(int idx) const;
    virtual const std::shared_ptr<cSimObj> &GetRootPart() const;
    virtual std::shared_ptr<cSimObj> GetRootPart();

    virtual void RegisterContacts(int contact_flag, int filter_flag);
    virtual bool HasFallen() const;
    virtual bool HasStumbled() const;
    virtual bool HasExploded() const;

    virtual bool IsInContact() const;
    virtual bool IsInContact(int idx) const;
    virtual tVector GetContactPt(int idx) const;

    virtual int GetState() const;
    virtual double GetPhase() const;

    virtual bool IsValidBodyPart(int idx) const;
    virtual bool EnableBodyPartFallContact(int idx) const;

    virtual void SetController(std::shared_ptr<cCharController> ctrl);
    virtual void RemoveController();
    virtual bool HasController() const;
    virtual const std::shared_ptr<cCharController> &GetController();
    virtual const std::shared_ptr<cCharController> &GetController() const;
    virtual void EnableController(bool enable);

    virtual void ApplyControlForces(const Eigen::VectorXd &tau);
    virtual void PlayPossum();

    virtual tVector GetCurrentGroundTarget();
    virtual void SetCurrentGroundTarget(tVector newGroundTarget);

#if defined(ENABLE_TRAINING)
    virtual double CalcEffort() const;
    virtual void ClearEffortBuffer();
#endif

    virtual tVector GetPartColor(int part_id) const;
    virtual bool HasDrawShapes() const;

    virtual const std::shared_ptr<cWorld> &GetWorld() const;

  protected:
    tVector currentGroundTarget;
    std::shared_ptr<cWorld> mWorld;
    std::vector<std::shared_ptr<cSimObj>> mBodyParts;
    std::vector<cJoint, Eigen::aligned_allocator<cJoint>> mJoints;
    Eigen::MatrixXd mBodyDefs;
    Eigen::MatrixXd mDrawShapeDefs;
    double mFriction;

    std::shared_ptr<cCharController> mController;

#if defined(ENABLE_TRAINING)
    // effort is the sum of squared torques over time
    std::vector<double> mEffortBuffer;
#endif
    virtual bool BuildSimBody(const tParams &params, const tVector &root_pos);
    virtual bool LoadBodyDefs(const std::string &char_file, Eigen::MatrixXd &out_body_defs) const;
    virtual bool LoadDrawShapeDefs(const std::string &char_file, Eigen::MatrixXd &out_draw_defs) const;

    virtual void BuildBodyPart(int part_id, const tVector &root_pos, double friction,
                               std::shared_ptr<cSimObj> &out_part);
    virtual void BuildBoxPart(int part_id, const tVector &root_pos, double friction,
                              std::shared_ptr<cSimObj> &out_part);
    virtual void BuildCapsulePart(int part_id, const tVector &root_pos, double friction,
                                  std::shared_ptr<cSimObj> &out_part);
    virtual void BuildConstraints();
    virtual void BuildConsFactor(int joint_id, tVector &out_linear_factor, tVector &out_angular_factor) const;
    virtual void BuildRootConsFactor(cKinTree::eJointType joint_type, tVector &out_linear_factor,
                                     tVector &out_angular_factor) const;

    virtual void ClearJointTorques();
    virtual void EnableJoints();
    virtual void ClearBodyForces();
    virtual void UpdateJoints();

    virtual short GetPartColGroup(int part_id) const;
    virtual short GetPartColMask(int part_id) const;

    virtual void BuildJointPose(int joint_id, Eigen::VectorXd &out_pose) const;
    virtual void BuildJointVel(int joint_id, Eigen::VectorXd &out_pose) const;

    virtual void BuildPose(Eigen::VectorXd &out_pose) const;
    virtual void BuildVel(Eigen::VectorXd &out_vel) const;

#if defined(ENABLE_TRAINING)
    virtual void UpdateEffortBuffer(double time_step);
#endif
};
