package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase

object SwerveSubsystem : SubsystemBase("Swerve subsystem") {
	//Converts from WPILib's 0 to 180 and then -180 to 0 with the positive side of the y axis rotation system
	//to the standard 0 to 360 with the positive side of the x axis rotation system used with vectors
	private fun convertRotationFromWPILibToStandard(angle: Rotation2d): Rotation2d =
		Rotation2d.fromDegrees((450.0 + angle.degrees) % 360)


}