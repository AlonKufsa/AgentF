package frc.robot.vision

import com.hamosad1657.lib.vision.AprilTagCamera
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.vision.VisionConstants as Constants

object AprilTagVision : AprilTagCamera("AprilTag-Cam") {
	override val maxTagTrustingDistance = Constants.MAX_TRUSTING_DISTANCE

	override val stdDevs: AprilTagsStdDevs
		get() = Constants.STD_DEVS

	override val robotToCamera: Transform3d
		get() = Constants.CAMERA_TO_ROBOT

	override val isAutonomousSupplier: () -> Boolean
		get() = { DriverStation.isAutonomous() }
}