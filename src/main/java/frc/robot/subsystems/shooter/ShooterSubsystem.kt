package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.minus
import com.hamosad1657.lib.units.rotations
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import kotlin.math.absoluteValue
import kotlin.math.cos
import frc.robot.subsystems.shooter.ShooterConstants as Constants

object ShooterSubsystem : SubsystemBase("Shooter") {


	//Needs to rotate counterclockwise, remember to check motor rotation direction is correct and invert if not
	//This is the leader motor, only this motor is given voltage and the other motor will be treated accordingly
	private val mainShootingMotor = HaSparkFlex(RobotMap.ShooterMap.SHOOTER_TOP_SHOOTING_MOTOR_ID, kBrushless).apply {
		pidController.apply {
			p = Constants.shooterMotorPIDGains.kP
			i = Constants.shooterMotorPIDGains.kI
			d = Constants.shooterMotorPIDGains.kD
		}
	}

	//Needs to rotate clockwise
	private val bottomShootingMotor =
		HaSparkFlex(RobotMap.ShooterMap.SHOOTER_BOTTOM_SHOOTING_MOTOR_ID, kBrushless).apply {
			follow(mainShootingMotor, true) //Follows the top motor, but inverts it's rotation direction
		}

	private val angleMotor = HaTalonFX(RobotMap.ShooterMap.SHOOTER_ANGLE_MOTOR_ID).apply {
		restoreFactoryDefaults()
		inverted = false
		configPID(Constants.shooterAnglePIDGains)
		with(configurator) {
			apply(Constants.ANGLE_CURRENT_LIMIT_CONFIGS)
			apply(Constants.ANGLE_FEEDBACK_CONFIGS)
			apply(Constants.ANGLE_MOTION_MAGIC_CONFIGS)
		}
	}

	private val canCoder = CANcoder(RobotMap.ShooterMap.CANCODER_ID).apply {
		//Makes the CANCoder measure in a range between 0 and 1, gives it the correct offset
		//and defines it's rotation direction as counterclockwise (which is flipped due to how the robot is built)
		configurator.apply(Constants.CANCODER_CONFIGS)
	}
	private val minAngleSwitch = DigitalInput(RobotMap.ShooterMap.MIN_SWITCH_ID)
	private val maxAngleSwitch = DigitalInput(RobotMap.ShooterMap.MAX_SWITCH_ID)

	//A shortcut for getting the current rotation from the cancoder
	val currentAngle: Rotation2d get() = canCoder.absolutePosition.value.rotations

	//A shortcut for getting the current speed of the shooter motors
	val currentSpeed: AngularVelocity get() = AngularVelocity.fromRpm(mainShootingMotor.encoder.velocity)

	//**Angle controller motor code**

	//Shortcuts for getting the position of the limit switches
	val isAtMinAngle get() = minAngleSwitch.get()
	val isAtMaxAngle get() = maxAngleSwitch.get()

	//A set of settings which are used to update the motor talonFX's setpoint, feed forward and such.
	//Changes to the setpoint and feed forward first need to be applied to this, and then this set of
	//settings is applied to the controller
	private val controlRequestShooterAngle = MotionMagicVoltage(0.0).apply { EnableFOC = false }

	//A variable storing the current setpoint for error calculations and such
	private var angleSetpoint: Rotation2d = 0.0.rotations

	val isWithinTolerance: Boolean get() = calcError().degrees.absoluteValue < 1.0

	/*
	* * F U N C T I O N  S U M M A R Y * *
	* This is the main function of the shooter movement system *
	Purpose:
	1. Allow the user to choose to update the setpoint.
	2. Update the FF, and stop the motor when it is at it's movement limits.
	3. Generally be a command that runs in loops and takes care of all the motor control and safety needed.

	How:
	   The function takes an optional new setpoint, clamps it in case it is outside the motion range,
	   then updates the setpoint and FF, and induces safety operations before applying the
	   updates to the motor controller.
	 */
	fun updateAngleControl(newSetpoint: Rotation2d = angleSetpoint) {
		if (newSetpoint.degrees in Constants.MIN_ANGLE.degrees..Constants.MAX_ANGLE.degrees) {
			//In the case that the new setpoint the user had sent is not in the motion range of the shooter,
			//it will clamp it and print an error to the driver station.
			DriverStation.reportError("Setpoint is not in motion range!", true)
			angleSetpoint =
				clamp(newSetpoint.degrees, Constants.MIN_ANGLE.degrees, Constants.MAX_ANGLE.degrees).rotations
		} else angleSetpoint = newSetpoint

		//Updates the changes that were made to the setpoint and calculates a new FF according to the current angle.
		controlRequestShooterAngle.apply {
			Position = angleSetpoint.rotations
			FeedForward = calculateAngleFF(currentAngle)
		}
		if (isLimited()) {
			//if the motor is at it's movement and wants to move beyond them,
			//it will get only the voltage needed to stay in place.
			angleMotor.setVoltage(controlRequestShooterAngle.FeedForward)
		} else {
			//if its within it's movement limits or at it's limits and wants to move to back into it's movement limits,
			//it will apply the changes to the setpoint and FF to the motor controller.
			angleMotor.setControl(controlRequestShooterAngle)
		}
	}

	//Calculates the FF needed for a given angle.
	private fun calculateAngleFF(currentAngle: Rotation2d): Volts {
		val floorRelativeAngle = currentAngle minus Constants.FLOOR_RELATIVE_OFFSET
		val ff: Volts = cos(floorRelativeAngle.radians) * Constants.KEEP_PARALLEL_TO_FLOOR_OUTPUT * 12.0
		return if (currentAngle.degrees < Constants.RESTING_ANGLE.degrees) ff * 1.125 - 0.1
		else ff + 0.125
	}

	//Calculates the error, needed to know what direction the shooter is moving to.
	private fun calcError(): Rotation2d = angleSetpoint minus currentAngle

	private fun isLimited(): Boolean {
		return (isAtMinAngle && (calcError().degrees < 0.0)) || (isAtMaxAngle && (calcError().degrees > 0.0))
	}
/////

	//Adds values and readings to the glass dashboard, attached to the shooter subsystem.
	override fun initSendable(builder: SendableBuilder) {
		builder.addDoubleProperty("Current angle deg", { currentAngle.degrees }, null)
		builder.addDoubleProperty("angle setpoint deg", { angleSetpoint.degrees }, null)
		builder.addDoubleProperty("Angle error", { calcError().degrees }, null)
		builder.addBooleanProperty("Is at max limit", { isAtMaxAngle }, null)
		builder.addBooleanProperty("Is at min limit", { isAtMinAngle }, null)
	}


	//**Shooting motors code**
	private var velocitySetpoint: AngularVelocity = AngularVelocity.fromRpm(0.0)


	//Updates the shooter motors PID controller that governs their angular velocity and updates FF
	fun updateShootingControl(newVelocitySetpoint: AngularVelocity) {
		velocitySetpoint = newVelocitySetpoint
		mainShootingMotor.pidController.setReference(velocitySetpoint.asRpm, ControlType.kVelocity)
	}

	fun setShooterState(angleSetpoint: Rotation2d, shooterVelocitySetpoint: AngularVelocity) {
		updateShootingControl(shooterVelocitySetpoint)
		updateAngleControl(angleSetpoint)
	}
}