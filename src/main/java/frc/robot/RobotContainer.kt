package frc.robot

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants.OperatorConstants
import frc.robot.commands.FeedToShooter
import frc.robot.commands.ForkCommand
import frc.robot.commands.TeleopDriveCommand
import frc.robot.commands.arm.ArmShootInAmpCommand
import frc.robot.commands.elevator.ManualElevatorControlCommand
import frc.robot.commands.intake.IntakeCommand
import frc.robot.commands.intake.IntakeFromHuman
import frc.robot.commands.intake.IntakeReverseCommand
import frc.robot.commands.intake.IntakeToArmCommand
import frc.robot.commands.led.LEDIdleCommand
import frc.robot.commands.led.LEDNoteInShooterCommand
import frc.robot.commands.led.LEDNoteIntakenCommand
import frc.robot.commands.led.LEDNoteToArmCommand
import frc.robot.commands.shooter.*
import frc.robot.subsystems.*
import frc.robot.subsystems.LEDSubsystem.LEDColor
import frc.robot.utils.DashboardTrigger
import kotlin.math.abs
import kotlin.math.sign


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

    enum class MatchState {
        NORMAL, END_GAME
    }

    var elevatorManual = ManualMode.LOCKED
    private var shooterManual = ManualMode.LOCKED
    private var matchMode = MatchState.NORMAL
    var isNormalMode = Trigger { matchMode == MatchState.NORMAL }
    var armManual = ManualMode.LOCKED

    lateinit var robotState: RobotState
    var useVision: Boolean = true
        private set

    var alliance: DriverStation.Alliance? = null
        private set

    // driver controller and triggers
    private val driverController = CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT)
    private val autoShootToggle = driverController.povRight()

    private val slowModeToggle = driverController.rightTrigger()
    private val intakeButton = driverController.rightBumper()
    private val resetHeading = driverController.b()
    private val ferryShot = driverController.leftBumper().or(DashboardTrigger("ferryShot"))
    private val visionAlign = driverController.povLeft().or(DashboardTrigger("visionAlign"))


    // operator controller and triggers
    private val operatorController = CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT)

    private val toggleNormalModeButton = operatorController.back().and(operatorController.start()) // test this
    private val reverseIntakeButton = operatorController.povDown().or(DashboardTrigger("reverseIntake"))
    private val manualIntakeButton = operatorController.povUp().or(DashboardTrigger("manualIntake"))
    private val intakeFromHumanButton = operatorController.a().or(DashboardTrigger("intakeFromHuman"))
    private val fireNoteToAmp = operatorController.y().or(DashboardTrigger("noteToAmp"))
    private val subwooferShot = operatorController.leftBumper().or(DashboardTrigger("subwooferShot"))
    private val podiumShot = operatorController.rightBumper().or(DashboardTrigger("podiumShot"))
    private val elevatorAndArmHomeButton = operatorController.x().or(DashboardTrigger("elevatorHome"))
    // FOR PS5 CONTROLLERS
//    private val anywhereShot =
//        operatorController.button(OperatorConstants.OPERATOR_LEFT_TRIGGER_ID).or(DashboardTrigger("anywhereShot"))
//    private val intakeToShooter = operatorController.button(OperatorConstants.OPERATOR_RIGHT_TRIGGER_ID)
//    private val unlockElevatorControl = operatorController.button(OperatorConstants.OPERATOR_LEFT_STICK_BUTTON_ID)
//    private val unlockShooterControl = operatorController.button(OperatorConstants.OPERATOR_RIGHT_STICK_BUTTON_ID)

    private val anywhereShot = operatorController.leftTrigger().or(DashboardTrigger("anywhereShot"))
    private val intakeToShooter = operatorController.rightTrigger()
    private val unlockElevatorControl = operatorController.leftStick()
    private val unlockShooterControl = operatorController.rightStick()

    fun getNoteDir(): String {
        val retrievedEntry = NetworkTableInstance.getDefault().getTable("vision").getEntry("noteDirection").getString("")
        return retrievedEntry
    }

    // this thread should run ONCE
    private val initializeDSRequiredTasks: Thread = Thread {
        while (!DriverStation.isDSAttached()) {
            DriverStation.reportWarning("attaching DS...", false)
        }
        DriverStation.reportWarning("DS attached", false)

        alliance = DriverStation.getAlliance().get()
        DriverStation.silenceJoystickConnectionWarning(true)

        configureTriggers() // configure triggers here so all threads are up-to-date when this is called
        SwerveSubsystem.initializePoseEstimator()
        LEDSubsystem.defaultCommand =
            LEDIdleCommand(if (alliance == DriverStation.Alliance.Red) LEDColor.RED else LEDColor.BLUE)

        // keep here so we can be sure that alliance is not null
        anywhereShot
            .and(isNormalMode)
            .whileTrue(
                ParallelCommandGroup(
                    RampShooter(
                        3000.0,
                        3000.0,
                        SwerveSubsystem.odometryImpl.getPivotAngle(alliance!!)
                    ),
                )
            )
    }

    init {
        initializeDSRequiredTasks.start()
        configureDefaultCommands()
        configureDriverBindings()
        configureOperatorBindings()
        configureEndGameBindings()
    }

    /**
     * Use this method to define your `trigger->command` mappings. Triggers can be created via the
     * [Trigger] constructor that takes a [BooleanSupplier][java.util.function.BooleanSupplier]
     * with an arbitrary predicate, or via the named factories in [GenericHID][edu.wpi.first.wpilibj2.command.button.CommandGenericHID]
     * subclasses such for [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureDriverBindings() {
        autoShootToggle.onTrue(InstantCommand({
            ShooterSubsystem.autoShootingEnabled = !ShooterSubsystem.autoShootingEnabled
        })
            .andThen(
                ConditionalCommand(
                    LEDSubsystem.flashCommand(LEDColor.WHITE, 0.2, 2.0),
                    LEDSubsystem.flashCommand(LEDColor.CYAN, 0.2, 2.0)
                ) { ShooterSubsystem.autoShootingEnabled }
            ))

        visionAlign.onTrue(
            InstantCommand({ useVision = !useVision })
        )

        slowModeToggle.whileTrue(InstantCommand({ SwerveSubsystem.toggleSpeedChange() })) // right trigger
        intakeButton.whileTrue(IntakeCommand()) // right bumper
        ferryShot.whileTrue(FerryShot()) // left bumper

        resetHeading.onTrue(
            Commands.runOnce(SwerveSubsystem::zeroHeading)
        )
    }

    private fun configureOperatorBindings() {
        toggleNormalModeButton.onTrue(
            SequentialCommandGroup(
                Commands.runOnce({
                    LEDSubsystem.defaultCommand.cancel()
                    LEDSubsystem.removeDefaultCommand()
                }),
                ConditionalCommand(
                    Commands.runOnce({
                        matchMode = MatchState.END_GAME
                        LEDSubsystem.defaultCommand = LEDSubsystem.fadeCommand(LEDColor.ORANGE)
                    }),
                    Commands.runOnce({
                        matchMode = MatchState.END_GAME
                        LEDSubsystem.defaultCommand =
                            LEDIdleCommand(if (alliance == DriverStation.Alliance.Red) LEDColor.RED else LEDColor.BLUE)
                    }),
                ) { matchMode == MatchState.NORMAL }
            )
        )

        unlockElevatorControl
            .and(isNormalMode)
            .onTrue(ElevatorSubsystem.getManualElevatorCommand(isNormalMode, operatorController))

        unlockShooterControl
            .and(isNormalMode)
            .onTrue(ConditionalCommand(
                InstantCommand({
                    shooterManual = ManualMode.UNLOCKED
                    ShooterSubsystem.disable()
                    ShooterSubsystem.defaultCommand.cancel()
                    ShooterSubsystem.defaultCommand =
                        ManualShooterPivotCommand {
                            if (!isNormalMode.asBoolean || abs(operatorController.getRawAxis(OperatorConstants.OPERATOR_RIGHT_STICK_AXIS)) < Constants.Deadbands.SHOOTER_DEADBAND) {
                                null
                            } else {
                                operatorController.getRawAxis(OperatorConstants.OPERATOR_RIGHT_STICK_AXIS).sign > 0
                            }
                        }
                }),
                InstantCommand({
                    shooterManual = ManualMode.LOCKED
                    ShooterSubsystem.enable()
                    ShooterSubsystem.defaultCommand.cancel()
                    ShooterSubsystem.defaultCommand =
                        ShooterHomePositionCommand()
                })
            ) { shooterManual == ManualMode.LOCKED })

//        unlockArmControl.onTrue(InstantCommand({ // right stick BUTTON
//            armManual = if (armManual == ManualMode.UNLOCKED) {
//                ManualMode.LOCKED
//            } else {
//                ManualMode.UNLOCKED
//            }
//        }))

        intakeToShooter // TEST THIS!!! // REMOVE THE TRIGGER THAT TURNS ON THE FEEDERS IN CONFIGURETRIGGERS
            .and(isNormalMode)
            .whileTrue(
                FeedToShooter()
            )
            .onTrue(
                LEDSubsystem.flashCommand(LEDColor.WHITE, 0.2, 2.0)
            )


        fireNoteToAmp // triangle || y // intake to arm
            .and(isNormalMode)
            .and { ElevatorSubsystem.atElevatorMin }
            .onTrue(
                IntakeToArmCommand().withTimeout(5.0)
                    .andThen(
                        SequentialCommandGroup(
                            Commands.runOnce(ArmSubsystem::goToAmpFirePosition),
                            WaitCommand(.5),
                            ElevatorSubsystem.getAmpCommand(),
                        ).unless { IntakeSubsystem.intakeToArmInterrupted }
                    )
            )

        fireNoteToAmp // shoot note
            .and(isNormalMode)
            .and { ArmSubsystem.armInPosition(ArmSubsystem.Position.AMP_SHOOT) }
            .and { ElevatorSubsystem.elevatorInPosition(ElevatorSubsystem.ElevatorLevel.AMP) }
            .onTrue(
                ParallelCommandGroup(
                    SequentialCommandGroup(
                        WaitCommand(.2),
                        ArmShootInAmpCommand(),
                    ),
                    LEDSubsystem.flashCommand(LEDColor.ORANGE, 0.2, 2.0)
                )
            )

        elevatorAndArmHomeButton
            .and(isNormalMode)
            .and { ElevatorSubsystem.elevatorInPosition(ElevatorSubsystem.ElevatorLevel.AMP) }
            .onTrue(
                SequentialCommandGroup(
                    InstantCommand({ ArmSubsystem.goToDangle() }),
                    WaitCommand(.2),
                    ElevatorSubsystem.getHomeCommand(),
                    ConditionalCommand(
                        InstantCommand({ ArmSubsystem.goHome() }),
                        InstantCommand(),
                        ElevatorSubsystem::atElevatorMin
                    )
                )
            )

        subwooferShot
            .and(isNormalMode)
            .whileTrue( // left bumper
                SubwooferShot()
            )

        podiumShot
            .and(isNormalMode)
            .whileTrue( // right bumper
                PodiumShot()
            )

        reverseIntakeButton
            .and(isNormalMode)
            .whileTrue(IntakeReverseCommand())

        manualIntakeButton
            .and(isNormalMode)
            .whileTrue(
                Commands.runEnd(
                    IntakeSubsystem::intake,
                    IntakeSubsystem::stop,
                    IntakeSubsystem
                )
            )

        intakeFromHumanButton
            .and(isNormalMode)
            .whileTrue(
                IntakeFromHuman()
            ).onTrue(LEDSubsystem.flashCommand(LEDColor.GREEN, .2, 2.0))
            .onFalse(Commands.runOnce({
                IntakeSubsystem.stop()
                ShooterSubsystem.stopFeed()
            }))
    }

    private fun configureEndGameBindings() {
        operatorController.y()
            .and(isNormalMode.negate())
            .and { elevatorManual == ManualMode.LOCKED }
            .and { !ElevatorSubsystem.isEnabled }
            .onTrue(
                SequentialCommandGroup(
                    Commands.runOnce({ ArmSubsystem.goToClimbPosition() }),
                    WaitCommand(.5),
                    ElevatorSubsystem.getClimbCommand()
                )
            )

        operatorController.rightStick()
            .and(isNormalMode.negate())
            .onTrue(ElevatorSubsystem.getManualElevatorCommand(isNormalMode, operatorController))
    }



    private fun configureDefaultCommands() {
        ShooterSubsystem.defaultCommand = ShooterHomePositionCommand()
        SwerveSubsystem.defaultCommand = TeleopDriveCommand(
            { -driverController.leftY },
            { -driverController.leftX },
            { driverController.rightX },
            { true }, // field relative
            { driverController.hid.leftTriggerAxis > 0.5 || anywhereShot.asBoolean }, // experimental code to face shooter while ramping "|| rampAnywhereButton.asBoolean"
            { driverController.hid.leftBumper }) // ferryShot.asBoolean
    }

    fun resetElevatorOnStart() { // PLEASE TEST AND BE MINDFUL OF FIRST
        if (ElevatorSubsystem.atElevatorMin) {
            DriverStation.reportWarning("Elevator good and can reset", false)
            ArmSubsystem.goHome()
        } else {
            DriverStation.reportWarning("ELEVATOR IS NOT RESET... Resetting to Home Now", false)
            Commands.runOnce(ArmSubsystem::goToDangle)
                .andThen(
                    WaitUntilCommand { ArmSubsystem.armInPosition(ArmSubsystem.Position.DANGLE) },
                    Commands.runOnce(ElevatorSubsystem::getHomeCommand)
                        .andThen(
                            ElevatorSubsystem::resetCANCoder
                        )
                )
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
                LEDNoteToArmCommand(LEDColor.PURPLE)
            )

        Trigger {
            IntakeSubsystem.noteStatus == NoteStatus.IN_SHOOTER
        }
            .whileTrue(
                LEDNoteInShooterCommand(LEDColor.MAGENTA)
            )

//        Trigger { // find out a fix to this
//            ShooterSubsystem.isShooterWithinRPMPercentageSetpoint(.75)
//        }.and(
//            ferryShot.or(podiumShot).or(subwooferShot).or(anywhereShot)
//        ) //.and(intakeToShooter.negate())
//            .whileTrue(
//                FeedToShooter()
//            ).onFalse(
//                Commands.runOnce({
//                    ShooterSubsystem.stopFeed()
//                })
//            )

        Trigger {
            ShooterSubsystem.firing
        }.and { ShooterSubsystem.isReadyToShoot }
            .and { ShooterSubsystem.autoShootingEnabled }
            .and(ferryShot.or(podiumShot).or(subwooferShot))
            .onTrue(
                SequentialCommandGroup(
                    WaitCommand(.25),
                    FeedToShooter().withTimeout(1.0)
//                    ForkCommand(IntakeSubsystem.IntakeDirection.TO_SHOOTER).withTimeout(1.0)
                )
            )

        Trigger {
            ShooterSubsystem.isReadyToShoot
        }.and(
            ferryShot
                .or(podiumShot)
                .or(subwooferShot)
        )
            .onTrue(
                RepeatCommand(
                    ShooterSubsystem.shooterLEDCommand()
                ).withTimeout(2.0)
                    .deadlineWith(
                        Commands.run({
                            operatorController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
                        })
                    )
            )
            .onFalse(Commands.run({
                operatorController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
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
