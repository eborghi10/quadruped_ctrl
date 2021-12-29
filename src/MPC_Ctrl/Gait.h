#ifndef PROJECT_GAIT_H
#define PROJECT_GAIT_H

#include <string>
#include <queue>

#include "Utilities/cppTypes.h"


class Gait {
public:
  virtual ~Gait() = default;

  virtual void setGaitParam(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string& name = "walk") = 0;
  virtual Vec4<float> getContactState() = 0;
  virtual Vec4<float> getSwingState() = 0;
  virtual int* getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;
  virtual float getCurrentStanceTime(float dtMPC, int leg) = 0;
  virtual float getCurrentSwingTime(float dtMPC, int leg) = 0;
  virtual float getCurrentGaitPhase() = 0;
  virtual int getGaitHorizon() = 0;
  virtual void debugPrint() { }

protected:
  std::string _name;
};

using Eigen::Array4f;
using Eigen::Array4i;

class OffsetDurationGait : public Gait {
public:
  OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  ~OffsetDurationGait();
  void setGaitParam(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name) override;
  Vec4<float> getContactState() override;
  Vec4<float> getSwingState() override;
  int* getMpcTable() override;
  void setIterations(int iterationsBetweenMPC, int currentIteration) override;
  float getCurrentStanceTime(float dtMPC, int leg) override;
  float getCurrentSwingTime(float dtMPC, int leg) override;
  float getCurrentGaitPhase() override;
  int getGaitHorizon() override;
  void debugPrint();

private:
  int* _mpc_table = NULL;
  Array4i _offsets; // offset in mpc segments
  Array4i _durations; // duration of step in mpc segments
  Array4f _offsetsFloat; // offsets in phase (0 to 1)
  Array4f _durationsFloat; // durations in phase (0 to 1)
  // int _stance;
  // int _swing;
  int _iteration;
  int _nIterations;
  float _phase;
};



class MixedFrequncyGait : public Gait {
public:
  MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string& name);
  ~MixedFrequncyGait();
  void setGaitParam(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  Vec4<float> getContactState() override;
  Vec4<float> getSwingState() override;
  int* getMpcTable() override;
  void setIterations(int iterationsBetweenMPC, int currentIteration) override;
  float getCurrentStanceTime(float dtMPC, int leg) override;
  float getCurrentSwingTime(float dtMPC, int leg) override;
  float getCurrentGaitPhase() override;
  int getGaitHorizon() override;
  void debugPrint();

private:
  float _duty_cycle;
  int* _mpc_table;
  Array4i _periods;
  Array4f _phase;
  int _iteration;
  int _nIterations;
};

#endif //PROJECT_GAIT_H
