package frc.robot.commands

import com.hamosad1657.lib.robotPrint
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.subsystems.swerve.SwerveKinematics
import frc.robot.subsystems.swerve.SwerveSubsystem

fun SwerveSubsystem.resetSwerveSetpoints(): Command {
	return run {

	}
}