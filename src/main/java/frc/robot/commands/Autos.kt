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
//    private val autoModeChooser = SendableChooser<AutoMode>().apply {
//        AutoMode.values().forEach { addOption(it.optionName, it) }
//        setDefaultOption(AutoMode.default.optionName, AutoMode.default)
//    }

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
        get() = AutoMode.entries[getAutonSelector].toString()

    /** Example static factory for an autonomous command.  */
    private fun exampleAuto(): Command =
        Commands.sequence(ExampleSubsystem.exampleMethodCommand(), ExampleCommand())

    private fun exampleAuto2() = PrintCommand("An example Auto Mode that just prints a value")

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
        CUSTOM_AUTO_1("Custom Auto Mode 1", exampleAuto()),
        CUSTOM_AUTO_2("Custom Auto Mode 2", exampleAuto2()),
        CUSTOM_AUTO_3("Custom Auto Mode 3", ExampleCommand()),
        CUSTOM_AUTO_4("Custom Auto Mode 4", testAuto()),
        CUSTOM_AUTO_5("Custom Auto Mode 5", testAutoShootThenMoveForward()),
        PATHPLANNER_AUTO_1("Path Planner 1", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test Path")))
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
            val default = CUSTOM_AUTO_4
        }
    }
}