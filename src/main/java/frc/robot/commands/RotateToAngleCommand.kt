package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.lib.util.math.MathConstants
import frc.robot.Constants
import frc.robot.subsystems.SwerveSubsystem

class RotateToAngleCommand(private val getRequestedAngle: () -> Double, private val getButtonPressed: () -> Boolean = { true }) : Command() {
    private val swerveSubsystem = SwerveSubsystem
    private val alignPID = PIDController(
        Constants.Swerve.AUTO_ALIGN_KP,
        Constants.Swerve.AUTO_ALIGN_KI.toDouble(),
        Constants.Swerve.AUTO_ALIGN_KD.toDouble()
    )

    init {
        alignPID.enableContinuousInput(0.0, 360.0)
        alignPID.setTolerance(1.0)


        addRequirements(swerveSubsystem)
    }

    override fun initialize() {
        val requestedAngle = getRequestedAngle()

        val robotHeading = MathConstants.continuous180To360(SwerveSubsystem.heading.degrees)
        val setpoint = (robotHeading + requestedAngle) % 360.0

        alignPID.setpoint = setpoint
    }

    override fun execute() {
        swerveSubsystem.drive(
            Translation2d(),
            (if (swerveSubsystem.isLowGear) 5 else 1) * alignPID.calculate(MathConstants.continuous180To360(SwerveSubsystem.heading.degrees)),
            fieldRelative = true,
            isOpenLoop = false,
        )
    }

    override fun isFinished(): Boolean {
        val buttonPressed = getButtonPressed()
        return if (buttonPressed == null) alignPID.atSetpoint()
        else ((alignPID.atSetpoint() && !buttonPressed)
                || !buttonPressed)
    }

    override fun end(interrupted: Boolean) {
        swerveSubsystem.stop()
    }
}
