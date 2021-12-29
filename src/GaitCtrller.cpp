#include "GaitCtrller.h"

#include "Dynamics/MiniCheetah.h"


GaitCtrller::GaitCtrller(double freq, double* PIDParam)
    : controlParameters{ new RobotControlParameters() }
{
  for (int i = 0; i < 4; i++) {
    ctrlParam(i) = PIDParam[i];
  }
  _gamepadCommand.resize(4);

  LoadParametersFromYAML(userParameters, "mini-cheetah-user-parameters.yaml");
  LoadParametersFromYAML(*controlParameters, "mini-cheetah-control-parameters.yaml");

  // Build the appropriate Quadruped object
  _quadruped = buildMiniCheetah<float>();
  control_data._quadruped = &_quadruped;

  // Always initialize the leg controller and state entimator
  control_data._legController = new LegController<float>(_quadruped);
  control_data._stateEstimator = new StateEstimatorContainer<float>(
      cheaterState.get(), &_vectorNavData, control_data._legController->datas, &_stateEstimate,
      controlParameters.get());

  // Initialize estimator
  control_data._stateEstimator->removeAllEstimators();
  control_data._stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  control_data._stateEstimator->setContactPhase(contactDefault);
  control_data._stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  control_data._stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();

  // Initialize the DesiredStateCommand object
  _desiredStateCommand = new DesiredStateCommand<float>(1.0 / freq);
  control_data._desiredStateCommand = _desiredStateCommand;

  // Initialize a new GaitScheduler object
  control_data._gaitScheduler = new GaitScheduler<float>(&userParameters, 1.0 / freq);
  control_data.controlParameters = controlParameters.get();
  control_data.userParameters = &userParameters;
  currentState = std::make_unique<FSM_State_Locomotion<float>>(&control_data);

  safetyChecker = std::make_unique<SafetyChecker<float>>();

  currentState->onEnter();

  std::cout << "finish init controller" << std::endl;
}

GaitCtrller::~GaitCtrller() {}

void GaitCtrller::LoadParametersFromYAML(ControlParameters& parameters, const std::string& filename) {
  printf("[SimControlPanel] Init simulator parameters...\n");
  parameters.initializeFromYamlFile(getConfigDirectoryPath() + filename);
  if (!parameters.isFullyInitialized()) {
    printf(
        "[ERROR] Simulator parameters are not fully initialized. You forgot: "
        "\n%s\n",
        parameters.generateUnitializedList().c_str());
    throw std::runtime_error("simulator not initialized");
  } else {
    printf("\tsim parameters are all good\n");
  }
}

void GaitCtrller::SetIMUData(double* imuData) {
  _vectorNavData.accelerometer(0, 0) = imuData[0];
  _vectorNavData.accelerometer(1, 0) = imuData[1];
  _vectorNavData.accelerometer(2, 0) = imuData[2];
  _vectorNavData.quat(0, 0) = imuData[3];
  _vectorNavData.quat(1, 0) = imuData[4];
  _vectorNavData.quat(2, 0) = imuData[5];
  _vectorNavData.quat(3, 0) = imuData[6];
  _vectorNavData.gyro(0, 0) = imuData[7];
  _vectorNavData.gyro(1, 0) = imuData[8];
  _vectorNavData.gyro(2, 0) = imuData[9];
}

void GaitCtrller::SetLegData(double* motorData) {
  for (int i = 0; i < 4; i++) {
    _legdata.q_abad[i] = motorData[i * 3];
    _legdata.q_hip[i] = motorData[i * 3 + 1];
    _legdata.q_knee[i] = motorData[i * 3 + 2];
    _legdata.qd_abad[i] = motorData[12 + i * 3];
    _legdata.qd_hip[i] = motorData[12 + i * 3 + 1];
    _legdata.qd_knee[i] = motorData[12 + i * 3 + 2];
  }
}

void GaitCtrller::PreWork(double* imuData, double* motorData) {
  SetIMUData(imuData);
  SetLegData(motorData);
  // Run the state estimator step
  control_data._stateEstimator->run();
  control_data._legController->updateData(&_legdata);
}

void GaitCtrller::SetGaitType(int gaitType) {
  _gaitType = gaitType;
  std::cout << "set gait type to: " << _gaitType << std::endl;
}

void GaitCtrller::SetRobotMode(int mode) {
  _robotMode = mode;
  std::cout << "set robot mode to: " << _robotMode << std::endl;
}

void GaitCtrller::SetRobotVel(double* vel) {
  _gamepadCommand[0] = abs(vel[0]) < 0.03 ? 0.0 : vel[0];
  _gamepadCommand[1] = abs(vel[1]) < 0.03 ? 0.0 : vel[1];
  _gamepadCommand[2] = abs(vel[2]) < 0.03 ? 0.0 : vel[2];
}

void GaitCtrller::TorqueCalculator(double* imuData, double* motorData,
                                   double* effort) {
  PreWork(imuData, motorData);

  // Setup the leg controller for a new iteration
  control_data._legController->zeroCommand();
  control_data._legController->setEnabled(true);
  control_data._legController->setMaxTorqueCheetah3(208.5);

  // Find the current gait schedule
  control_data._gaitScheduler->step();

  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands(_gamepadCommand);

  //safety check
  if(!safetyChecker->checkSafeOrientation(*control_data._stateEstimator)){
    _safetyCheck = false;
    std::cout << "broken: Orientation Safety Check FAIL" << std::endl;

  }else if (!safetyChecker->checkPDesFoot(_quadruped, *control_data._legController)) {
    _safetyCheck = false;
    std::cout << "broken: Foot Position Safety Check FAIL" << std::endl;

  }else if (!safetyChecker->checkForceFeedForward(*control_data._legController)) {
    _safetyCheck = false;
    std::cout << "broken: Force FeedForward Safety Check FAIL" << std::endl;

  }else if (!safetyChecker->checkJointLimit(*control_data._legController)) {
    _safetyCheck = false;
    std::cout << "broken: Joint Limit Safety Check FAIL" << std::endl;
  }

  // Run the Control FSM code
  // _controlFSM->runFSM();
  currentState->run();

  control_data._legController->updateCommand(&legcommand, ctrlParam);

  if(_safetyCheck) {
    for (int i = 0; i < 4; i++) {
      effort[i * 3] = legcommand.tau_abad_ff[i];
      effort[i * 3 + 1] = legcommand.tau_hip_ff[i];
      effort[i * 3 + 2] = legcommand.tau_knee_ff[i];
    }
  } else {
    for (int i = 0; i < 4; i++) {
      effort[i * 3] = 0.0;
      effort[i * 3 + 1] = 0.0;
      effort[i * 3 + 2] = 0.0;
    }
  }

  // return effort;
}
