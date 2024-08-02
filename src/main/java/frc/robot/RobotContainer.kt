package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants.OperatorConstants
import frc.robot.commands.Autos
import frc.robot.commands.led.LEDIdleCommand
import frc.robot.commands.led.LEDNoteIntakenCommand
import frc.robot.commands.TeleopDriveCommand
import frc.robot.commands.arm.ArmShootInAmpCommand
import frc.robot.commands.elevator.ManualElevatorControlCommand
import frc.robot.commands.intake.IntakeCommand
import frc.robot.commands.intake.IntakeFromPlayer
import frc.robot.commands.intake.IntakeReverseCommand
import frc.robot.commands.intake.IntakeToArmCommand
import frc.robot.commands.led.LEDNoteInShooterCommand
import frc.robot.commands.led.LEDNoteToArmCommand
import frc.robot.commands.shooter.*
import frc.robot.subsystems.*
import frc.robot.utils.DashboardTrigger
import kotlin.math.truncate


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
    private var shooterManual = ManualMode.LOCKED
    private var armManual = ManualMode.LOCKED

    lateinit var robotState: RobotState
    var useVision: Boolean = true
        private set

    var alliance: DriverStation.Alliance? = null
        private set

    // driver controller and triggers
    private val driverController = CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)
    private val driverModifierKey = driverController.povRight()

    private val slowModeToggle = driverController.rightTrigger()
    private val intakeButton = driverController.rightBumper()
    private val resetHeading = driverController.b()
    private val unlockElevatorControl = driverController.leftStick()
    private val elevatorUp = driverController.povUp()
    private val elevatorDown = driverController.povDown()
    private val ferryShot = driverController.leftBumper().or(DashboardTrigger("ferryShot"))
    private val visionAlign = driverController.povLeft().or(DashboardTrigger("visionAlign"))


    // operator controller and triggers
    private val operatorController = CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT)

    private val reverseIntakeButton = operatorController.povDown().or(DashboardTrigger("reverseIntake"))
    private val manualIntakeButton = operatorController.povUp().or(DashboardTrigger("manualIntake"))

    private val intakeFromHumanButton = operatorController.a().or(DashboardTrigger("intakeFromHuman"))

    private val fireNoteToAmp = operatorController.y().or(DashboardTrigger("fireNoteToAmp"))
    private val subwooferShot = operatorController.leftBumper().or(DashboardTrigger("subwooferShot"))
    private val podiumShot = operatorController.rightBumper().or(DashboardTrigger("podiumShot"))
    private val unlockShooterControl = operatorController.leftStick()
    private val unlockArmControl = operatorController.rightStick()


    // this thread should run ONCE
    private val valueGetter = Thread {
        while (!DriverStation.isDSAttached()) {
            DriverStation.reportWarning("Attaching DS...", false)
        }
        DriverStation.reportWarning("DS Attached", false)

        alliance = DriverStation.getAlliance().get()

        SwerveSubsystem.initializePoseEstimator() // configure pose on thread

        LEDSubsystem.defaultCommand =
            LEDIdleCommand(if (alliance == DriverStation.Alliance.Red) LEDSubsystem.LEDColor.RED else LEDSubsystem.LEDColor.BLUE)
        configureTriggers() // configure triggers here so all threads are up to date when this is called
        DriverStation.reportWarning("Current alliance is $alliance", false)
    }

    init {
        valueGetter.start()

        configureDefaultCommands()
        configureDriverBindings()
        configureOperatorBindings()
        Autos.configureNamedCommands()
    }

    /**
     * Use this method to define your `trigger->command` mappings. Triggers can be created via the
     * [Trigger] constructor that takes a [BooleanSupplier][java.util.function.BooleanSupplier]
     * with an arbitrary predicate, or via the named factories in [GenericHID][edu.wpi.first.wpilibj2.command.button.CommandGenericHID]
     * subclasses such for [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureDriverBindings() {
        unlockElevatorControl.onTrue(InstantCommand({ // left stick BUTTON
            elevatorManual = if (elevatorManual == ManualMode.UNLOCKED) {
                ManualMode.LOCKED
            } else {
                ManualMode.UNLOCKED
            }
        }))

        visionAlign.onTrue(
            InstantCommand({ useVision = !useVision }))

        slowModeToggle.whileTrue(InstantCommand({ SwerveSubsystem.toggleSpeedChange() })) // right trigger
//        reverseIntakeButton.whileTrue(IntakeReverseCommand()) // right bumper && modifier key (right dpad)
        intakeButton.whileTrue(IntakeCommand()) // right bumper

        resetHeading.onTrue(
            Commands.runOnce(SwerveSubsystem::zeroHeading))

        elevatorUp // dpad up
            .or(elevatorDown) // dpad down
            .and { elevatorManual == ManualMode.UNLOCKED }
            .whileTrue(ManualElevatorControlCommand { driverController.hid.pov == 180 })

        ferryShot.whileTrue(FerryShot()) // left bumper
    }

    private fun configureOperatorBindings() {
        unlockShooterControl.onTrue(InstantCommand({ // left stick BUTTOn
            shooterManual = if (shooterManual == ManualMode.UNLOCKED) {
                ManualMode.LOCKED
            } else {
                ManualMode.UNLOCKED
            }
        }))

        unlockArmControl.onTrue(InstantCommand({ // right stick BUTTON
            armManual = if (armManual == ManualMode.UNLOCKED) {
                ManualMode.LOCKED
            } else {
                ManualMode.UNLOCKED
            }
        }))

        fireNoteToAmp // triangle || y
            .and { IntakeSubsystem.noteStatus != NoteStatus.ARM }
            .and { ElevatorSubsystem.atElevatorMin }
            .onTrue(
                IntakeToArmCommand().withTimeout(5.0)
            )

        fireNoteToAmp // triangle || y
            .and { IntakeSubsystem.noteStatus == NoteStatus.ARM }
            .onTrue(
                ArmShootInAmpCommand()
            )

        subwooferShot.whileTrue( // left bumper
            SubwooferShot()
        )

        podiumShot.whileTrue( // right bumper
            PodiumShot()
        )

        reverseIntakeButton.whileTrue(IntakeReverseCommand())
        manualIntakeButton.whileTrue(
            IntakeCommand()
//            Commands.runEnd(
//                IntakeSubsystem::intake,
//                IntakeSubsystem::stop,
//                IntakeSubsystem
//            )
        )

        intakeFromHumanButton.whileTrue(IntakeFromPlayer())
    }

    private fun configureDefaultCommands() {
        ShooterSubsystem.defaultCommand = ShooterHomePositionCommand()
        SwerveSubsystem.defaultCommand = TeleopDriveCommand(
            { -driverController.leftY },
            { -driverController.leftX },
            { driverController.rightX },
            { true }, // field relative
            { driverController.hid.leftTriggerAxis > 0.5 },
            { ferryShot.asBoolean })


        if (ElevatorSubsystem.atElevatorMin) {
            println("Elevator good and can reset");
            ArmSubsystem.goHome()
        } else {
            DriverStation.reportWarning("ELEVATOR IS NOT RESET... Resetting to Home Now", false)
            /** DO NOT UNCOMMENT THIS UNTIL THE ELEVATOR IS FIXED!!!!!!! */
//            Commands.runOnce(ArmSubsystem::goToDangle)
//                .andThen(
//                    WaitUntilCommand { ArmSubsystem.armInPosition(ArmSubsystem.Position.DANGLE) },
//                    Commands.runOnce(ElevatorSubsystem::getHomeCommand)
//                )
//                .schedule()
        }
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

        Trigger {
            IntakeSubsystem.noteStatus == NoteStatus.IN_SHOOTER
        }
            .whileTrue(
                LEDNoteInShooterCommand(LEDSubsystem.LEDColor.MAGENTA)
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
        }.and(ferryShot
            .or(podiumShot)
            .or(subwooferShot))
            .onTrue(
                RepeatCommand(
                    ShooterSubsystem.shooterLEDCommand().alongWith(
//                        RepeatCommand(InstantCommand({ NetworkTables.isReadyToShoot.set(true) }))
                    )
                ).withTimeout(2.0)
                    .deadlineWith(Commands.run({
                        driverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
                        operatorController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
                    })
                )
            )
            .onFalse(Commands.run({
                driverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
                operatorController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
//                InstantCommand({ NetworkTables.isReadyToShoot.set(false) })
            }))

        Trigger {
            ElevatorSubsystem.atElevatorMin
        }.or(ElevatorSubsystem::atElevatorMax)
            .onTrue(
                ElevatorSubsystem.elevatorLEDCommand()
            )
    }

    fun resetControllerRumble() {
        driverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        operatorController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
    }
}
