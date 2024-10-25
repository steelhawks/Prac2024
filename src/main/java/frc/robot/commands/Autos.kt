package frc.robot.commands

import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.commands.intake.IntakeCommand
import frc.robot.commands.shooter.ManualShotCommand
import frc.robot.commands.shooter.PodiumShot
import frc.robot.commands.shooter.SubwooferShot
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ShooterSubsystem
import frc.robot.subsystems.SwerveSubsystem

object Autos {
    init {
        configureNamedCommands()
    }

    /** The commands used by PathPlanner */

    private fun configureNamedCommands() {
        NamedCommands.registerCommand("intake", IntakeCommand())

        NamedCommands.registerCommand(
            "subwoofer shot",
            ParallelRaceGroup(
                WaitCommand(2.0),
                ParallelDeadlineGroup(
                    SequentialCommandGroup(
                        WaitUntilCommand(ShooterSubsystem::isReadyToShoot),
                        ParallelDeadlineGroup(
                            WaitCommand(.3),
                            FeedToShooter()
                        )
                    ),
                    SubwooferShot()
                )
            )
        )

        NamedCommands.registerCommand(
            "anywhere shot",
            ParallelRaceGroup(
                WaitCommand(4.0),
                ParallelDeadlineGroup(
                    SequentialCommandGroup(
                        WaitUntilCommand(ShooterSubsystem::isReadyToShoot),
                        ParallelDeadlineGroup(
                            WaitCommand(0.3),
                            ForkCommand(IntakeSubsystem.IntakeDirection.TO_SHOOTER)
                        )
                    ),
                    RotateToAngleCommand(
                        {
                            SwerveSubsystem.calculateTurnAngle(
                                if (RobotContainer.alliance == DriverStation.Alliance.Red) Constants.BlueTeamPoses.BLUE_SPEAKER_POSE else Constants.RedTeamPoses.RED_SPEAKER_POSE,
                                SwerveSubsystem.heading.degrees + 180
                            )
                        }
                    ),
                    Commands.parallel(
                        Commands.run(
                            {ShooterSubsystem.setGoal(SwerveSubsystem.odometryImpl.getPivotAngle(RobotContainer.alliance!!))}
                        ),
                        Commands.runEnd(
                            {ShooterSubsystem.rampShooter(3000.0, 3000.0)},
                            {ShooterSubsystem.stopShooter()}
                        ),
                        Commands.runEnd(
                            {ShooterSubsystem.feedToShooter()},
                            {ShooterSubsystem.stopFeed()}
                        )
                    )

                )
            )
        )

        NamedCommands.registerCommand(
            "anywhere ramp",
            Commands.parallel(
                Commands.run(
                    {ShooterSubsystem.setGoal(SwerveSubsystem.odometryImpl.getPivotAngle(RobotContainer.alliance!!))}
                ),
                Commands.runEnd(
                    {ShooterSubsystem.rampShooter(1700.0, 1700.0)},
                    {ShooterSubsystem.stopShooter()}
                ),
                Commands.runEnd(
                    {ShooterSubsystem.feedToShooter()},
                    {ShooterSubsystem.stopFeed()}
                )
            )
        )

        NamedCommands.registerCommand(
            "anywhere feed",
            ParallelRaceGroup(
                WaitCommand(1.0),
                SequentialCommandGroup(
                    WaitUntilCommand(ShooterSubsystem::isReadyToShoot),
                    ParallelDeadlineGroup(
                        WaitCommand(0.3),
                        ForkCommand(IntakeSubsystem.IntakeDirection.TO_SHOOTER)
                    )
                )
            )
        )
    }

    private val autonSelector: Array<DigitalInput> = arrayOf(
        DigitalInput(10),
        DigitalInput(11),
        DigitalInput(12),
        DigitalInput(13),
        DigitalInput(18),
        DigitalInput(19),
        DigitalInput(20),
        DigitalInput(21),
        DigitalInput(22),
        DigitalInput(23),
        DigitalInput(24)
    )

    val getAutonSelector: Int
        get() {
            for (i in autonSelector.indices) {
                if (!autonSelector[i].get()) {
                    return i
                }
            }

            return 0
        }

    val defaultAutonomousCommand: Command
        get() = AutoMode.default.command

    val selectedAutonomousCommand: Command
        get() = AutoMode[getAutonSelector]

    val selectedAutonomousUseVision: Boolean
        get() = AutoMode.entries[getAutonSelector].useVision

    val selectedAutonomousCommandName: String
        get() = AutoMode.entries[getAutonSelector].optionName

    /** Auton Static Factories  */

    private fun nothingAuto(): Command {
        return PrintCommand("This selection has no auto set")
    }

    /**
     * An enumeration of the available autonomous modes.
     *
     * @param optionName The name for the PathPlanner Auto option.
     * @param useVision The boolean to use vision or not.
     */
    private enum class AutoMode(val optionName: String, val useVision: Boolean) {
        AUTO_1("4 piece", false),
        AUTO_2("3 note center bottom", false),
        AUTO_3("4 piece reverse", false),
        AUTO_4("2 note center bottom", true),
        AUTO_5("5 piece top", false),
        AUTO_6("5 piece top 2", true),
        AUTO_7("6 piece", true),
        AUTO_8("3 note center top", true),
        AUTO_9("Amp-Side 4 Piece", false),
        AUTO_10("4 piece heitman", false),
        AUTO_11("nothing", false);

        val command: Command by lazy {
            if (this == AUTO_11) nothingAuto() else PathPlannerAuto(optionName)
        }

        companion object {
            operator fun get(autonSelector: Int): Command {
                return entries.getOrNull(autonSelector)?.command ?: defaultAutonomousCommand
            }

            /** The default auto mode. */
            val default = AUTO_11
        }
    }
}
