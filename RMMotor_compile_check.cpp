// Compile-only coverage for the public RMMotor interface and CAN placement
// rules. This function is never called; the module CI only needs the
// declarations to compile.
#include <type_traits>

#include "RMMotor.hpp"

namespace rmmotor_compile_check {
void CompilePublicInterface() {
  [[maybe_unused]] static void *null_can = nullptr;
  auto &can = *static_cast<std::remove_reference_t<LibXR::CAN &> *>(null_can);

  static RMMotor motor_1(can, {.model = RMMotor::Model::MOTOR_M3508,
                               .reverse = false,
                               .feedback_id = 0x201});
  static RMMotor motor_2(can, {.model = RMMotor::Model::MOTOR_M3508,
                               .reverse = false,
                               .feedback_id = 0x202});
  static RMMotor motor_3(can, {.model = RMMotor::Model::MOTOR_M3508,
                               .reverse = false,
                               .feedback_id = 0x203});
  static RMMotor motor_4(can, {.model = RMMotor::Model::MOTOR_M3508,
                               .reverse = false,
                               .feedback_id = 0x204});
  static RMMotor m2006(can, {.model = RMMotor::Model::MOTOR_M2006,
                             .reverse = false,
                             .feedback_id = 0x205});
  static RMMotor gm6020(can, {.model = RMMotor::Model::MOTOR_GM6020,
                              .reverse = false,
                              .feedback_id = 0x209});

  motor_1.Enable();
  motor_1.Disable();
  motor_1.Relax();
  motor_1.ClearError();
  motor_1.SaveZeroPoint();
  (void)motor_1.Update();
  (void)motor_1.GetFeedback();
  (void)motor_1.GetOmega();
  motor_1.Control({.mode = Motor::ControlMode::MODE_CURRENT, .velocity = 0.0f});
  motor_1.Control({.mode = Motor::ControlMode::MODE_TORQUE,
                   .reduction_ratio = 1.0f,
                   .torque = 0.0f});
  motor_1.TorqueControl(0.0f, 1.0f);

  Motor *base = &motor_1;
  base->Relax();
  (void)motor_2;
  (void)motor_3;
  (void)motor_4;
  (void)m2006;
  (void)gm6020;
}
} // namespace rmmotor_compile_check

static_assert(RMMotor::DerivePlacement(RMMotor::Model::MOTOR_M3508, 0x201)
                  .config.id_control == 0x200);
static_assert(
    RMMotor::DerivePlacement(RMMotor::Model::MOTOR_M3508, 0x204).slot == 3);
static_assert(RMMotor::DerivePlacement(RMMotor::Model::MOTOR_M3508, 0x205)
                  .config.id_control == 0x1FF);
static_assert(RMMotor::DerivePlacement(RMMotor::Model::MOTOR_GM6020, 0x205)
                  .config.id_control == 0x1FE);
static_assert(RMMotor::DerivePlacement(RMMotor::Model::MOTOR_GM6020, 0x209)
                  .config.id_control == 0x2FE);
static_assert(RMMotor::DerivePlacement(RMMotor::Model::MOTOR_M3508, 0x200)
                  .config.id_control == 0);
static_assert(RMMotor::KGetTorque(RMMotor::Model::MOTOR_M3508) == 0.0156224f);
static_assert(RMMotor::GetCurrentMAX(RMMotor::Model::MOTOR_M3508) == 20.0f);
static_assert(RMMotor::GetLSB(RMMotor::Model::MOTOR_M3508) == 16384.0f);
