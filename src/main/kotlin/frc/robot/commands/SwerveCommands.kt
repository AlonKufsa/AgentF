package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.swerve.SwerveSubsystem

fun SwerveSubsystem.setSwerveRotation(rotation: () -> Rotation2d): Command {
	return run { setRotation(rotation()) }
}

fun SwerveSubsystem.setSwerveDriveVoltage(voltage: () -> Volts): Command {
	return run { setDriveVoltage(voltage()) } finallyDo { setDriveVoltage(0.0) }
}

fun SwerveSubsystem.setSwerveSpeedMPS(speed: () -> Double): Command {
	return run { setSpeed(speed()) }
}