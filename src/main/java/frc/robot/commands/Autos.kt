package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.*
import frc.robot.commands.intake.IntakeCommand
import frc.robot.commands.shooter.ManualShotCommand
import frc.robot.commands.shooter.SubwooferShot
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ShooterSubsystem

object Autos
{
    /** The commands used by PathPlanner */

    fun configureNamedCommands() {
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
                                ForkCommand(IntakeSubsystem.IntakeDirection.TO_INTAKE)
                            )
                        )
                    ),
                    SubwooferShot()
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
     * way to manage all our autonomous modes. The [autoModeChooser] iterates
     * over its values, adding each value to the chooser.
     *
     * @param optionName The name for the [autoModeChooser] option.
     * @param command The [Command] to run for this mode.
     */
    @Suppress("unused")
    private enum class AutoMode(val optionName: String, val command: Command, val useVision: Boolean)
    {
        AUTO_1("Fire and Rotate", testAuto(), false),
        AUTO_2("Fire and Move Forward", testAutoShootThenMoveForward(), false),
        AUTO_3("Path Planner Test", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test Path")), false),
        AUTO_4("new auto pathplanner", PathPlannerAuto("test auto"), false),
        AUTO_5("simple path test pathplanner", PathPlannerAuto("home auto"), true),
        AUTO_6("Placeholder Auto 3", nothingAuto(), false),
        AUTO_7("Placeholder Auto 4", nothingAuto(), false),
        AUTO_8("Placeholder Auto 5", nothingAuto(), false),
        AUTO_9("Placeholder Auto 6", nothingAuto(), false),
        AUTO_10("Placeholder Auto 7", nothingAuto(), false),
        AUTO_11("Placeholder Auto 8", nothingAuto(), false)
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
            val default = AUTO_1
        }
    }
}
