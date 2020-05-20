/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <ocs2_core/Types.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_ddp/SLQ.h>
#include <ocs2_oc/test/EXP0.h>

using namespace ocs2;

enum { STATE_DIM = 2, INPUT_DIM = 1 };

TEST(exp0_slq_test, exp0_slq_test) {
  SLQ_Settings slqSettings;
  slqSettings.useNominalTimeForBackwardPass_ = false;
  slqSettings.ddpSettings_.preComputeRiccatiTerms_ = true;
  slqSettings.ddpSettings_.displayInfo_ = true;
  slqSettings.ddpSettings_.displayShortSummary_ = true;
  slqSettings.ddpSettings_.absTolODE_ = 1e-10;
  slqSettings.ddpSettings_.relTolODE_ = 1e-7;
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 10000;
  slqSettings.ddpSettings_.maxNumIterations_ = 30;
  slqSettings.ddpSettings_.minRelCost_ = 5e-4;
  slqSettings.ddpSettings_.checkNumericalStability_ = true;
  slqSettings.ddpSettings_.useFeedbackPolicy_ = true;
  slqSettings.ddpSettings_.debugPrintRollout_ = false;
  slqSettings.ddpSettings_.strategy_ = ddp_strategy::type::LINE_SEARCH;
  slqSettings.ddpSettings_.lineSearch_.minStepLength_ = 0.0001;

  Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-10;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 10000;

  // event times
  std::vector<scalar_t> eventTimes{0.1897};
  std::vector<size_t> subsystemsSequence{0, 1};
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr(new ModeScheduleManager({eventTimes, subsystemsSequence}));

  scalar_t startTime = 0.0;
  scalar_t finalTime = 2.0;

  // partitioning times
  std::vector<scalar_t> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(eventTimes[0]);
  partitioningTimes.push_back(finalTime);

  vector_t initState(2);
  initState << 0.0, 2.0;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // system rollout
  EXP0_System systemDynamics(modeScheduleManagerPtr);
  TimeTriggeredRollout timeTriggeredRollout(STATE_DIM, INPUT_DIM, systemDynamics, rolloutSettings);

  // system derivatives
  EXP0_SystemDerivative systemDerivative(modeScheduleManagerPtr);

  // system constraints
  EXP0_SystemConstraint systemConstraint(STATE_DIM, INPUT_DIM);

  // system cost functions
  EXP0_CostFunction systemCostFunction(modeScheduleManagerPtr);

  // system operatingTrajectories
  vector_t stateOperatingPoint = vector_t::Zero(2);
  vector_t inputOperatingPoint = vector_t::Zero(1);
  OperatingPoints operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // SLQ - single-thread version
  slqSettings.ddpSettings_.nThreads_ = 1;
  SLQ slqST(STATE_DIM, INPUT_DIM, &timeTriggeredRollout, &systemDerivative, &systemConstraint, &systemCostFunction, &operatingTrajectories,
            slqSettings);
  slqST.setModeScheduleManager(modeScheduleManagerPtr);

  slqSettings.ddpSettings_.nThreads_ = 3;
  // SLQ - multi-thread version
  SLQ slqMT(STATE_DIM, INPUT_DIM, &timeTriggeredRollout, &systemDerivative, &systemConstraint, &systemCostFunction, &operatingTrajectories,
            slqSettings);
  slqMT.setModeScheduleManager(modeScheduleManagerPtr);

  // run single core SLQ
  if (slqSettings.ddpSettings_.displayInfo_ || slqSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> single-core SLQ" << std::endl;
  }
  slqST.run(startTime, initState, finalTime, partitioningTimes);

  // run multi-core SLQ
  if (slqSettings.ddpSettings_.displayInfo_ || slqSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> multi-core SLQ" << std::endl;
  }
  slqMT.run(startTime, initState, finalTime, partitioningTimes);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // get solution
  PrimalSolution solutionST = slqST.primalSolution(finalTime);
  PrimalSolution solutionMT = slqMT.primalSolution(finalTime);

  // get performance indices
  auto performanceIndecesST = slqST.getPerformanceIndeces();
  auto performanceIndecesMT = slqMT.getPerformanceIndeces();

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const scalar_t expectedCost = 9.766;
  ASSERT_LT(fabs(performanceIndecesST.totalCost - expectedCost), 10 * slqSettings.ddpSettings_.minRelCost_)
      << "MESSAGE: single-threaded SLQ failed in the EXP1's cost test!";
  ASSERT_LT(fabs(performanceIndecesMT.totalCost - expectedCost), 10 * slqSettings.ddpSettings_.minRelCost_)
      << "MESSAGE: multi-threaded SLQ failed in the EXP1's cost test!";

  const scalar_t expectedISE1 = 0.0;
  ASSERT_LT(fabs(performanceIndecesST.stateInputEqConstraintISE - expectedISE1), 10 * slqSettings.ddpSettings_.constraintTolerance_)
      << "MESSAGE: single-threaded SLQ failed in the EXP1's type-1 constraint ISE test!";
  ASSERT_LT(fabs(performanceIndecesMT.stateInputEqConstraintISE - expectedISE1), 10 * slqSettings.ddpSettings_.constraintTolerance_)
      << "MESSAGE: multi-threaded SLQ failed in the EXP1's type-1 constraint ISE test!";

  const scalar_t expectedISE2 = 0.0;
  ASSERT_LT(fabs(performanceIndecesST.stateEqConstraintISE - expectedISE2), 10 * slqSettings.ddpSettings_.constraintTolerance_)
      << "MESSAGE: single-threaded SLQ failed in the EXP1's type-2 constraint ISE test!";
  ASSERT_LT(fabs(performanceIndecesMT.stateEqConstraintISE - expectedISE2), 10 * slqSettings.ddpSettings_.constraintTolerance_)
      << "MESSAGE: multi-threaded SLQ failed in the EXP1's type-2 constraint ISE test!";

  scalar_t ctrlFinalTime;
  if (slqSettings.ddpSettings_.useFeedbackPolicy_) {
    ctrlFinalTime = dynamic_cast<LinearController*>(solutionST.controllerPtr_.get())->timeStamp_.back();
  } else {
    ctrlFinalTime = dynamic_cast<FeedforwardController*>(solutionST.controllerPtr_.get())->timeStamp_.back();
  }
  ASSERT_DOUBLE_EQ(solutionST.timeTrajectory_.back(), finalTime) << "MESSAGE: SLQ_ST failed in policy final time of trajectory!";
  ASSERT_DOUBLE_EQ(ctrlFinalTime, finalTime) << "MESSAGE: SLQ_ST failed in policy final time of controller!";
}

TEST(exp0_slq_test, caching_test) {
  SLQ_Settings slqSettings;
  slqSettings.useNominalTimeForBackwardPass_ = false;
  slqSettings.ddpSettings_.displayInfo_ = false;
  slqSettings.ddpSettings_.displayShortSummary_ = true;
  slqSettings.ddpSettings_.absTolODE_ = 1e-10;
  slqSettings.ddpSettings_.relTolODE_ = 1e-7;
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 10000;
  slqSettings.ddpSettings_.maxNumIterations_ = 1;
  slqSettings.ddpSettings_.minRelCost_ = 5e-4;
  slqSettings.ddpSettings_.checkNumericalStability_ = true;
  slqSettings.ddpSettings_.useFeedbackPolicy_ = true;
  slqSettings.ddpSettings_.debugPrintRollout_ = false;
  slqSettings.ddpSettings_.debugCaching_ = true;  // for this test, debugCaching_ should be active
  slqSettings.ddpSettings_.nThreads_ = 1;         // single threaded
  slqSettings.ddpSettings_.strategy_ = ddp_strategy::type::LINE_SEARCH;
  slqSettings.ddpSettings_.lineSearch_.minStepLength_ = 0.0001;

  Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-10;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 10000;

  // switching times
  std::vector<scalar_t> eventTimes{1.0};
  std::vector<size_t> subsystemsSequence{0, 1};
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr(new ModeScheduleManager({eventTimes, subsystemsSequence}));

  // partitioning times
  std::vector<scalar_t> partitioningTimes;
  partitioningTimes.push_back(0.0);
  partitioningTimes.push_back(0.8);
  partitioningTimes.push_back(1.4);
  partitioningTimes.push_back(2.0);

  vector_t initState(2);
  initState << 0.0, 2.0;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // system rollout
  EXP0_System systemDynamics(modeScheduleManagerPtr);
  TimeTriggeredRollout timeTriggeredRollout(STATE_DIM, INPUT_DIM, systemDynamics, rolloutSettings);

  // system derivatives
  EXP0_SystemDerivative systemDerivative(modeScheduleManagerPtr);

  // system constraints
  EXP0_SystemConstraint systemConstraint(STATE_DIM, INPUT_DIM);

  // system cost functions
  EXP0_CostFunction systemCostFunction(modeScheduleManagerPtr);

  // system operatingTrajectories
  vector_t stateOperatingPoint = vector_t::Zero(2);
  vector_t inputOperatingPoint = vector_t::Zero(1);
  OperatingPoints operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // SLQ - single-thread version
  SLQ slqST(STATE_DIM, INPUT_DIM, &timeTriggeredRollout, &systemDerivative, &systemConstraint, &systemCostFunction, &operatingTrajectories,
            slqSettings);
  slqST.setModeScheduleManager(modeScheduleManagerPtr);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // run single core SLQ (no active event)
  scalar_t startTime = 0.2;
  scalar_t finalTime = 0.7;
  ASSERT_NO_THROW(slqST.run(startTime, initState, finalTime, partitioningTimes));

  // run similar to the MPC setup (a new partition)
  startTime = 0.4;
  finalTime = 0.9;
  ASSERT_NO_THROW(slqST.run(startTime, initState, finalTime, partitioningTimes, std::vector<ControllerBase*>()));

  // run similar to the MPC setup (one active event)
  startTime = 0.6;
  finalTime = 1.2;
  ASSERT_NO_THROW(slqST.run(startTime, initState, finalTime, partitioningTimes, std::vector<ControllerBase*>()));

  // run similar to the MPC setup (no active event + a new partition)
  startTime = 1.1;
  finalTime = 1.5;
  ASSERT_NO_THROW(slqST.run(startTime, initState, finalTime, partitioningTimes, std::vector<ControllerBase*>()));

  // run similar to the MPC setup (no overlap)
  startTime = 1.6;
  finalTime = 2.0;
  ASSERT_NO_THROW(slqST.run(startTime, initState, finalTime, partitioningTimes, std::vector<ControllerBase*>()));
}
