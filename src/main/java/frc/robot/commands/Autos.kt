package frc.robot.commands

import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.shooter.ManualShotCommand
import frc.robot.subsystems.ExampleSubsystem

object Autos
{
    private val autoModeChooser = SendableChooser<AutoMode>().apply {
        AutoMode.values().forEach { addOption(it.optionName, it) }
        setDefaultOption(AutoMode.default.optionName, AutoMode.default)
    }

    val defaultAutonomousCommand: Command
        get() = AutoMode.default.command

    val selectedAutonomousCommand: Command
        get() = autoModeChooser.selected?.command ?: defaultAutonomousCommand

    /** Example static factory for an autonomous command.  */
    private fun exampleAuto(): Command =
        Commands.sequence(ExampleSubsystem.exampleMethodCommand(), ExampleCommand())

    private fun exampleAuto2() = PrintCommand("An example Auto Mode that just prints a value")

    private fun testAuto(): Command {
        return SequentialCommandGroup(
            ManualShotCommand(),
            TeleopDriveCommand({0.0}, {0.0}, {0.2}, { true })
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
        // TODO: Replace with real auto modes and their corresponding commands
        CUSTOM_AUTO_1("Custom Auto Mode 1", exampleAuto()),
        CUSTOM_AUTO_2("Custom Auto Mode 2", exampleAuto2()),
        CUSTOM_AUTO_3("Custom Auto Mode 3", ExampleCommand()),
        CUSTOM_AUTO_4("Custom Auto Mode 4", testAuto()),
//        PATH_PLANNER_AUTO_4("Path Planner 4", PathPlannerAuto("Test")),
        ;

        companion object
        {
            /** The default auto mode. */
            val default = CUSTOM_AUTO_4
        }
    }
}