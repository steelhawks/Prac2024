package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants.OperatorConstants
import frc.robot.commands.Autos
import frc.robot.commands.led.LEDIdleCommand
import frc.robot.commands.led.LEDNoteIntakenCommand
import frc.robot.commands.TeleopDriveCommand
import frc.robot.commands.arm.ArmShootInAmpCommand
import frc.robot.commands.arm.ArmShootCommand
import frc.robot.commands.elevator.ManualElevatorControlCommand
import frc.robot.commands.intake.IntakeCommand
import frc.robot.commands.intake.IntakeReverseCommand
import frc.robot.commands.intake.IntakeToArmCommand
import frc.robot.commands.led.LEDNoteToArmCommand
import frc.robot.commands.shooter.*
import frc.robot.subsystems.*


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
object RobotContainer {
    enum class RobotState {
        DISABLED, TELEOP, AUTON, TEST
    }

    enum class ManualMode {
        LOCKED, UNLOCKED
    }

    private var elevatorManual = ManualMode.LOCKED

    lateinit var robotState: RobotState

    var alliance: DriverStation.Alliance? = null
        private set

    // driver controller and triggers
    private val driverController = CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)

    private val slowModeToggle = driverController.rightTrigger()
    private val reverseIntakeButton = driverController.leftBumper()
    private val intakeButton = driverController.rightBumper()
    private val resetHeading = driverController.b()

    // test triggers
    private val manualShooterButton = driverController.x()


    // operator controller and triggers
    private val operatorController = CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT)

    private val fireNoteToAmp = operatorController.y()

    // this thread should run ONCE
    private val valueGetter = Thread {
        while (!DriverStation.isDSAttached()) {
            println("Driver Station not attached")
            DriverStation.reportWarning("attaching DS...", false)
        }
        DriverStation.reportWarning("DS attached", false)

        alliance = DriverStation.getAlliance().get()

        SwerveSubsystem.initializePoseEstimator() // configure pose on thread

        LEDSubsystem.defaultCommand =
            LEDIdleCommand(if (alliance == DriverStation.Alliance.Red) LEDSubsystem.LEDColor.RED else LEDSubsystem.LEDColor.BLUE)
        configureTriggers() // configure triggers here so all threads are up to date when this is called
        println(alliance)
    }

    init {
        valueGetter.start()

        configureDefaultCommands()
        configureDriverBindings()
        configureOperatorBindings()
        // Reference the Autos object so that it is initialized, placing the chooser on the dashboard
        Autos
    }

    /**
     * Use this method to define your `trigger->command` mappings. Triggers can be created via the
     * [Trigger] constructor that takes a [BooleanSupplier][java.util.function.BooleanSupplier]
     * with an arbitrary predicate, or via the named factories in [GenericHID][edu.wpi.first.wpilibj2.command.button.CommandGenericHID]
     * subclasses such for [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureDriverBindings() {
        driverController.leftStick().onTrue(InstantCommand({
            elevatorManual = if (elevatorManual == ManualMode.UNLOCKED) {
                ManualMode.LOCKED
            } else {
                ManualMode.UNLOCKED
            }
        }))

        slowModeToggle.whileTrue(InstantCommand({ SwerveSubsystem.toggleSpeedChange() }))
        reverseIntakeButton.whileTrue(IntakeReverseCommand())
        intakeButton.whileTrue(IntakeCommand())

        resetHeading.onTrue(InstantCommand({
            SwerveSubsystem.zeroHeading()
        }))

        manualShooterButton.whileTrue(ManualShotCommand())

        driverController.povLeft().whileTrue(ArmShootCommand())

        driverController.povUp()
            .or(driverController.povDown())
            .and { elevatorManual == ManualMode.UNLOCKED }
            .whileTrue(ManualElevatorControlCommand { driverController.hid.pov == 180 })

        driverController.leftTrigger().whileTrue(FerryShot())
    }

    private fun configureOperatorBindings() {
        fireNoteToAmp
            .and { IntakeSubsystem.noteStatus != NoteStatus.ARM }
            .and { ElevatorSubsystem.atElevatorMin }
            .onTrue(
                IntakeToArmCommand().withTimeout(2.0)
            )

        fireNoteToAmp
            .and { IntakeSubsystem.noteStatus == NoteStatus.ARM }
            .onTrue(
                ArmShootInAmpCommand()
            )

        operatorController.leftBumper().whileTrue(
            SubwooferShot()
        )

        operatorController.rightBumper().whileTrue(
            PodiumShot()
        )
    }

    private fun configureDefaultCommands() {
        ArmSubsystem.goHome()

        ShooterSubsystem.defaultCommand = ShooterHomePositionCommand()

        if (ElevatorSubsystem.atElevatorMin) {
            println("Elevator good and can reset");
        } else {
            DriverStation.reportWarning("ELEVATOR IS NOT RESET... Resetting to Home Now", false)
        }

        SwerveSubsystem.defaultCommand = TeleopDriveCommand(
            { driverController.leftY },
            { driverController.leftX },
            { driverController.rightX },
            { true }) // field relative
    }

    private fun configureTriggers() {
        Trigger {
            IntakeSubsystem.intakeBeamBroken
        }
            .onTrue(
                IntakeSubsystem.intakeLEDCommand(alliance!!)
                    .andThen(
                        LEDNoteIntakenCommand(alliance!!)
                    )
                    .deadlineWith(
                        Commands.run({
                            driverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
                        })
                            .andThen(
                                Commands.runOnce({
                                    driverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
                                })
                            )
                    )
            ).onFalse(
                IntakeSubsystem.intakeLEDCommand(alliance!!)
                    .andThen(
                        Commands.runOnce({
                            driverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
                        })
                    )
            )

        Trigger {
            IntakeSubsystem.noteStatus == NoteStatus.ARM
        }
            .whileTrue(
                LEDNoteToArmCommand(LEDSubsystem.LEDColor.PURPLE)
            )

//        Trigger {
//            ShooterSubsystem.firing
//        }
//            .whileTrue(
//                RepeatCommand(
//                    ShooterSubsystem.shooterLEDCommand()
//                )
//            )

        Trigger {
            ShooterSubsystem.isReadyToShoot
        }.and(driverController.leftTrigger()
            .or(operatorController.leftBumper())
            .or(operatorController.rightBumper())
            .or(operatorController.leftTrigger())
            .or(operatorController.rightTrigger()))
            .onTrue(
                RepeatCommand(
                    ShooterSubsystem.shooterLEDCommand()
                ).withTimeout(2.0)
                    .deadlineWith(
                    Commands.run({
                        driverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
                        operatorController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
                    })
                )
            )
            .onFalse(Commands.run({
                driverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
                operatorController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
            }))
    }

    fun resetControllerRumble() {
        driverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        operatorController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
    }
}
