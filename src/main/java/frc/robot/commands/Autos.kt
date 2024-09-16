package frc.robot.commands

import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.*
import frc.robot.commands.intake.IntakeCommand
import frc.robot.commands.shooter.ManualShotCommand
import frc.robot.commands.shooter.PodiumShot
import frc.robot.commands.shooter.SubwooferShot
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ShooterSubsystem

object Autos
{
    init {
        configureNamedCommands()
    }

    /** The commands used by PathPlanner */

    private fun configureNamedCommands() {
        NamedCommands.registerCommand("intake", IntakeCommand())

        NamedCommands.registerCommand("subwoofer shot",
            ParallelRaceGroup(
                WaitCommand(2.0),
                ParallelDeadlineGroup(
                    SequentialCommandGroup(
                        WaitUntilCommand(ShooterSubsystem::isReadyToShoot),
                        ParallelDeadlineGroup(
                            WaitCommand(.3),
                            ParallelCommandGroup(
                                FeedToShooter(),
                            )
                        )
                    ),
                    SubwooferShot()
                )
            )
        )

        NamedCommands.registerCommand("podium shoot",
            ParallelRaceGroup(
                WaitCommand(4.0),
                ParallelDeadlineGroup(
                    SequentialCommandGroup(
                        WaitUntilCommand(ShooterSubsystem::isReadyToShoot),
                        ParallelDeadlineGroup(
                            WaitCommand(.3),
                            ParallelCommandGroup(
                                FeedToShooter(),
                                ForkCommand(IntakeSubsystem.IntakeDirection.TO_SHOOTER)
                            )
                        )
                    ),
                    PodiumShot()
                )
            ))

        NamedCommands.registerCommand("ramp from anywhere",
            ParallelRaceGroup(

            ))

        NamedCommands.registerCommand("shoot from anywhere",
            SequentialCommandGroup(
                WaitUntilCommand(ShooterSubsystem::isReadyToShoot),
                ParallelDeadlineGroup(
                    WaitCommand(.3),
                    ParallelCommandGroup(
                        FeedToShooter(),
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

    private fun testAuto(): Command {
        return SequentialCommandGroup(
            ManualShotCommand().withTimeout(2.0),
            TeleopDriveCommand({0.0}, {0.0}, {0.2}, { true }, getFaceSpeaker = { false }, getFaceAmp = { false }).withTimeout(5.0)
        )
    }

    private fun testAutoShootThenMoveForward(): Command {
        return SequentialCommandGroup(
            ManualShotCommand().withTimeout(1.0),
            TeleopDriveCommand({.2}, {0.0}, {0.0}, { true }, getFaceSpeaker = { false }, getFaceAmp = { false }).withTimeout(1.0)
        )
    }

    /**
     * An enumeration of the available autonomous modes. It provides an easy
     * way to manage all our autonomous modes. The [autonSelector] iterates
     * over its values, adding each value to the chooser.
     *
     * @param optionName The name for the PathPlanner Auto option.
     * @param command The [Command] to run for this mode.
     * @param useVision The boolean to use vision or not.
     */
    @Suppress("unused")
    private enum class AutoMode(val optionName: String, val command: Command, val useVision: Boolean)
    {
        AUTO_1("4 piece", PathPlannerAuto(AUTO_1.optionName), false),
        AUTO_2("3 note center bottom", PathPlannerAuto(AUTO_2.optionName),false),
        AUTO_3("4 piece reverse", PathPlannerAuto(AUTO_3.optionName), false),
        AUTO_4("2 note center bottom", PathPlannerAuto(AUTO_4.optionName), true),
        AUTO_5("5 piece top", PathPlannerAuto(AUTO_5.optionName),false),
        AUTO_6("5 piece top 2", PathPlannerAuto(AUTO_6.optionName), true),
        AUTO_7("6 piece", PathPlannerAuto(AUTO_7.optionName), true),
        AUTO_8("3 note center top", PathPlannerAuto(AUTO_8.optionName), true),
        AUTO_9("Amp-Side 4 Piece", PathPlannerAuto(AUTO_9.optionName), false),
        AUTO_10("4 piece heitman", PathPlannerAuto(AUTO_10.optionName), false),
        AUTO_11("nothing", nothingAuto(), false)
        ;

        companion object
        {
            operator fun get(autonSelector: Int): Command {
                var i = 0
                for (autoMode in entries) {
                    if (i != autonSelector) {
                        i++
                        continue
                    } else {
                        return autoMode.command
                    }
                }

                return defaultAutonomousCommand
            }

            /** The default auto mode. */
            val default = AUTO_11
        }
    }
}
