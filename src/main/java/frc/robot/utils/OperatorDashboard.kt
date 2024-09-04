package frc.robot.utils

import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
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
    private val alliance: StringPublisher = status.getStringTopic("alliance").publish()

    init {
        try {
            val ip = InetAddress.getLocalHost()
            println("IP ADDR NT4 " + ip.hostAddress + ":5810")
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    private var counter = 0
    override fun periodic() {
        counter = (counter + 1) % 1000

        if (counter % 10 == 0) { // runs this every 10 cycles every 200 ms
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

            if (RobotContainer.alliance != null) {
                alliance.set(if (RobotContainer.alliance == Alliance.Red) "Red" else "Blue")
            }

            robotState.set(RobotContainer.robotState.name)
            noteStatus.set(IntakeSubsystem.noteStatus.name)
            isReadyToShoot.set(ShooterSubsystem.isReadyToShoot)
        }
    }

    fun close() {
        robotState.close()
        noteStatus.close()
        isReadyToShoot.close()
        elevatorLevel.close()
        alliance.close()
    }
}
