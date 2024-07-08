package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants.OperatorConstants
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.commands.IntakeCommand
import frc.robot.commands.Autos
import frc.robot.commands.TeleopDriveCommand
import frc.robot.subsystems.SwerveSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer
{
    private val driverController = CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)
    private val operatorController = CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT)

    private val reverseIntakeButton = driverController.leftBumper()
    private val intakeButton = driverController.rightBumper()

    private val slowModeToggle = driverController.rightTrigger()
    init
    {
        configureBindings()
        // Reference the Autos object so that it is initialized, placing the chooser on the dashboard
        Autos

        SwerveSubsystem.defaultCommand = TeleopDriveCommand(
            { driverController.leftX },
            { driverController.leftY },
            { driverController.rightX })
    }

    /**
     * Use this method to define your `trigger->command` mappings. Triggers can be created via the
     * [Trigger] constructor that takes a [BooleanSupplier][java.util.function.BooleanSupplier]
     * with an arbitrary predicate, or via the named factories in [GenericHID][edu.wpi.first.wpilibj2.command.button.CommandGenericHID]
     * subclasses such for [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings()
    {
        slowModeToggle.whileTrue(InstantCommand({ SwerveSubsystem.toggleSpeedChange() }))
        reverseIntakeButton.whileTrue(InstantCommand({ IntakeSubsystem.intakeReverse() }))
        intakeButton.whileTrue(IntakeCommand())
    }
}
