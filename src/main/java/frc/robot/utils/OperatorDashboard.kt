package frc.robot.utils

import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer
import frc.robot.subsystems.ElevatorSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ShooterSubsystem
import java.net.InetAddress

object OperatorDashboard : SubsystemBase() {
    private var ntInstance: NetworkTableInstance = NetworkTableInstance.getDefault()
    private val status: NetworkTable = ntInstance.getTable("status")

    // state values to send to client these SHOULD be updated in periodic
    private val robotState: StringPublisher = status.getStringTopic("robotState").publish()
    private val noteStatus: StringPublisher = status.getStringTopic("noteStatus").publish()
    private val isReadyToShoot: BooleanPublisher = status.getBooleanTopic("isReadyToShoot").publish()
    private val elevatorLevel: StringPublisher = status.getStringTopic("elevatorLevel").publish()

    init {
        try {
            val ip = InetAddress.getLocalHost()
            println("IP ADDR NT4 " + ip.hostAddress + ":5810")
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    override fun periodic() {
        elevatorLevel.set(
            if (ElevatorSubsystem.atElevatorMax) {
                "Home"
            } else if (ElevatorSubsystem.elevatorInPosition(ElevatorSubsystem.ElevatorLevel.CLIMB)) {
                "Climb"
            } else if (ElevatorSubsystem.elevatorInPosition(ElevatorSubsystem.ElevatorLevel.AMP)) {
                "AMP"
            } else {
                ""
            }
        )

        robotState.set(RobotContainer.robotState.name)
        noteStatus.set(IntakeSubsystem.noteStatus.name)
        isReadyToShoot.set(ShooterSubsystem.isReadyToShoot)
    }

    fun close() {
        robotState.close()
        noteStatus.close()
    }
}
