#pragma once

namespace Types {

using CanId = int;

enum class CoordinateSystem {
  ROBOT,
  FIELD,
};

namespace Swerve {

  /**
   * Container for the four swerve modules in the drivetrain.
   * Values are ordered according to SetIndices.
   */
  template<typename Widget>
  using Set = std::array<Widget, 4>;

  /**
   * Aliases mapping the four corners to their indexed equivalents.
   */
  enum SetIndices {
    FL, FR, RL, RR,
  };

  /**
   * The motor controllers that each module needs.
   */
  template<typename Widget, typename U = Widget>
  struct MotorPair {
    Widget drive;
    U steer;
  };

}
}
