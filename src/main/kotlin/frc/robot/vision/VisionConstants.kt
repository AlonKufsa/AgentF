package frc.robot.vision

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.vision.AprilTagCamera.AprilTagsStdDevs
import com.hamosad1657.lib.vision.RobotPoseStdDevs
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import kotlin.math.PI

object VisionConstants {
	val rotateSwerveWithYawPIDGains = PIDGains(kP = 0.05)

	// AprilTag vision
	val MAX_TRUSTING_DISTANCE = Length.fromMeters(5)
	val CAMERA_TO_ROBOT =
		Transform3d(Translation3d(0.75 / 2 - 0.055, 0.75 / 2 - 0.06, 0.4), Rotation3d(0.0, (1.0 / 3.0) * PI, 0.0))
	val STD_DEVS = AprilTagsStdDevs(
		oneTag = RobotPoseStdDevs(1.2, 1.2, 9999.0),//(0.9, 0.9, 0.95),
		twoTagsAuto = RobotPoseStdDevs(0.9, 0.9, 9999.0),//(0.5, 0.5, 0.95),
		twoTagsTeleop = RobotPoseStdDevs(0.9, 0.9, 9999.0)//(0.35, 0.35, 0.95)
	)
}