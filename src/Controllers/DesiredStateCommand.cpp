/*========================= Gamepad Control ==========================*/
/**
 *
 */

#include "DesiredStateCommand.h"
/*=========================== Gait Data ===============================*/
/**
 *
 */
template <typename T>
void DesiredStateData<T>::zero() {
  // Overall desired state
  stateDes = Vec12<T>::Zero();
  stateTrajDes = Eigen::Matrix<T, 12, 10>::Zero();
}

template struct DesiredStateData<double>;
template struct DesiredStateData<float>;

/**
 *
 */
template <typename T>
void DesiredStateCommand<T>::convertToStateCommands() {
  data.zero();
  Vec2<float> joystickLeft, joystickRight;

  // No Remote Controller
  joystickLeft = gamepadCommand[0];   // gamepadCommand->leftStickAnalog;
  joystickRight = gamepadCommand[1];  // gamepadCommand->rightStickAnalog;
  // Warning!!!!
  // Recommend not to use stateDes
  // We are going to remove it soon

  joystickLeft[0] *= -1.f;
  joystickRight[0] *= -1.f;

  leftAnalogStick = leftAnalogStick * (T(1) - filter) + joystickLeft * filter;
  rightAnalogStick = rightAnalogStick * (T(1) - filter) + joystickRight * filter;

  // Desired states from the controller
  data.stateDes(6) = deadband(leftAnalogStick[1], minVelX, maxVelX);  // forward linear velocity
  data.stateDes(7) = deadband(leftAnalogStick[0], minVelY, maxVelY);  // lateral linear velocity
  data.stateDes(8) = 0.0;  // vertical linear velocity
  data.stateDes(0) = dt * data.stateDes(6);  // X position
  data.stateDes(1) = dt * data.stateDes(7);  // Y position
  data.stateDes(2) = 0.26;  // Z position height
  data.stateDes(9) = 0.0;  // Roll rate
  data.stateDes(10) = 0.0;  // Pitch rate
  data.stateDes(11) = deadband(rightAnalogStick[0], minTurnRate, maxTurnRate);  // Yaw turn rate
  data.stateDes(3) = 0.0; // Roll
  data.stateDes(4) = deadband(rightAnalogStick[1], minPitch, maxPitch);  // Pitch
  data.stateDes(5) = dt * data.stateDes(11);  // Yaw
}

template <typename T>
void DesiredStateCommand<T>::setCommandLimits(T minVelX_in, T maxVelX_in,
    T minVelY_in, T maxVelY_in, T minTurnRate_in, T maxTurnRate_in) {
  minVelX = minVelX_in;
  maxVelX = maxVelX_in;
  minVelY = minVelY_in;
  maxVelY = maxVelY_in;
  minTurnRate = minTurnRate_in;
  maxTurnRate = maxTurnRate_in;
}

/**
 *
 */
template <typename T>
void DesiredStateCommand<T>::desiredStateTrajectory(int N, Vec10<T> dtVec) {
  A = Mat12<T>::Zero();
  A(0, 0) = 1;
  A(1, 1) = 1;
  A(2, 2) = 1;
  A(3, 3) = 1;
  A(4, 4) = 1;
  A(5, 5) = 1;
  A(6, 6) = 1;
  A(7, 7) = 1;
  A(8, 8) = 1;
  A(9, 9) = 1;
  A(10, 10) = 1;
  A(11, 11) = 1;
  data.stateTrajDes.col(0) = data.stateDes;

  for (int k = 1; k < N; k++) {
    A(0, 6) = dtVec(k - 1);
    A(1, 7) = dtVec(k - 1);
    A(2, 8) = dtVec(k - 1);
    A(3, 9) = dtVec(k - 1);
    A(4, 10) = dtVec(k - 1);
    A(5, 11) = dtVec(k - 1);
    data.stateTrajDes.col(k) = A * data.stateTrajDes.col(k - 1);
  }
}

/**
 *
 */
template <typename T>
float DesiredStateCommand<T>::deadband(float command, T minVal, T maxVal) {
  if (command < deadbandRegion && command > -deadbandRegion) {
    return 0.0;
  } else {
    return (command / (2)) * (maxVal - minVal);
  }
}

template class DesiredStateCommand<double>;
template class DesiredStateCommand<float>;
