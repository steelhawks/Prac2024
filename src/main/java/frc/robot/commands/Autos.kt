package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.shooter.ManualShotCommand
import frc.robot.subsystems.ExampleSubsystem

object Autos
{
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

    val selectedAutonomousCommandName: String
        get() = AutoMode.entries[getAutonSelector].optionName

    /** Auton Static Factories  */

    private fun nothingAuto(): Command {
        return PrintCommand("This selection has no auto set")
    }

    private fun testAuto(): Command {
        return SequentialCommandGroup(
            ManualShotCommand().withTimeout(2.0),
            TeleopDriveCommand({0.0}, {0.0}, {0.2}, { true }).withTimeout(5.0)
        )
    }

    private fun testAutoShootThenMoveForward(): Command {
        return SequentialCommandGroup(
            ManualShotCommand().withTimeout(1.0),
            TeleopDriveCommand({.2}, {0.0}, {0.0}, { true }).withTimeout(1.0)
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
    private enum class AutoMode(val optionName: String, val command: Command)
    {
        AUTO_1("Fire and Rotate", testAuto()),
        AUTO_2("Fire and Move Forward", testAutoShootThenMoveForward()),
        AUTO_3("Path Planner Test", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test Path"))),
        AUTO_4("Placeholder Auto 1", nothingAuto()),
        AUTO_5("Placeholder Auto 2", nothingAuto()),
        AUTO_6("Placeholder Auto 3", nothingAuto()),
        AUTO_7("Placeholder Auto 4", nothingAuto()),
        AUTO_8("Placeholder Auto 5", nothingAuto()),
        AUTO_9("Placeholder Auto 6", nothingAuto()),
        AUTO_10("Placeholder Auto 7", nothingAuto()),
        AUTO_11("Placeholder Auto 8", nothingAuto())
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