# Haptic_manipulator
Manipulator with haptic feedback joystick.

period: 2022.07. ~ 2022.12.

---
### materials
Manipulator: robotis 사의 open_manipulator x 패키지 및 하드웨어

Joystick: 3d systems 사의 touch 패키지 및 하드웨어

---
### Info

조이스틱의 3차원 직교좌표 정보를 매니퓰레이터의 End Effector command로 인가

매니퓰레이터의 End Effector에 가해지는 외력을 조이스틱의 Force command로 인가

결론적으로, 사용자는 직관적이고 안전하게 매니퓰레이터를 조작할 수 있음.

---
### code brief

open_manipulator/open_manipulator_teleop/src/force.cpp

- 중력 매트릭스, 서보 전류엔코더 정보를 바탕으로 외력 추정

- 이를 바탕으로 조이스틱 force command 생성

<br>

open_manipulator/open_manipulator_teleop/src/try_s.cpp

- 조이스틱의 3차원 정보를 받아서 manipulator position command 생성

<br>

Geomagic_Touch_ROS_Drivers/omni_common/src/omni_state.cpp

- End Effector에 가해진 외력 추정값을 받아서 force를 생성.

- 조이스틱의 3차원 정보를 manipulator에 전달
