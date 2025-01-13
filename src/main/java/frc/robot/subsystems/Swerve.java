package frc.robot.subsystems;

// STEP 2: Add the epilogue @Logged annotation
// STEP 2: Extend Swerve with SubsystemBase
public class Swerve {

  // STEP 5: Add a prviate Pose2d estimatedPose. This should automatically get picked up for logging
  // by epilogue

  public Swerve() {

    // STEP 5: Initialize the Pose2d. An empty constructor will set the robot at (0,0) on the field,
    // facing to the right.

  }

  /**
   * Step 3a: Private Method driveFieldRelative that takes field-relative inputs
   *
   * @param X field-relative X with range of -1.0 to 1.0
   * @param Y field-relative Y with range of -1.0 to 1.0
   * @param Omega field-relative omega with range of -1.0 to 1.0
   */

  /**
   * Step 3b: Public command factory driveFieldRelativeCommand that takes field-relative inputs
   * and returns a command that passes the input parameters into the private driveFieldRelative
   * @param X field-relative X with range of -1.0 to 1.0
   * @param Y field-relative Y with range of -1.0 to 1.0
   * @param Omega field-relative omega with range of -1.0 to 1.0
   */

  // STEP 5: In the driveFieldRelative update the estimatedPose by scaling the input
  // parameters and adding them to the current pose values.

  // STEP 6: Use the MAX speeds constants to scale the field-relative inputs to update the
  // estimatedPose

}

