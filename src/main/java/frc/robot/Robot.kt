package frc.robot

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.PS4Controller
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import kotlin.math.cos

typealias Volts = Double

/**
 * The VM is configured to automatically run this object (which basically functions as a singleton class),
 * and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 * This is written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object Robot : TimedRobot() {

	val minSwitch = DigitalInput(0)
	val maxSwitch = DigitalInput(1)
	val ps4Controller = PS4Controller(0)

	val motorConfigs = TalonFXConfiguration().apply {
		Slot0.kP = 12.0
		Slot0.kI = 0.5
		Slot0.kD = 0.0
	}
	val motionMagicConfigs = MotionMagicConfigs().apply {
		MotionMagicCruiseVelocity = 2.0
		MotionMagicAcceleration = 4.0
	}
	val feedbackConfigs = FeedbackConfigs().apply {
		FeedbackSensorSource = RemoteCANcoder
		FeedbackRemoteSensorID = 3
	}
	val motor = TalonFX(4).apply {
		inverted = false
		with(configurator) {
			apply(motorConfigs)
			apply(motionMagicConfigs)
			apply(feedbackConfigs)
		}
	}
	val CANcoder = CANcoder(3)
	private val controlRequestShooterAngle = MotionMagicVoltage(0.0).apply { EnableFOC = false }

	private val RESTING_ANGLE = 223.5
	private const val KEEP_PARALLEL_TO_FLOOR_OUTPUT = -0.0185
	private val FLOOR_RELATIVE_OFFSET = RESTING_ANGLE - 90.0
	private var setpoint = 0.0


	fun calculateAngleFF(currentAngle: Double): Volts {
		val floorRelativeAngle = currentAngle - FLOOR_RELATIVE_OFFSET
		val ff = cos(Math.toRadians(floorRelativeAngle)) * KEEP_PARALLEL_TO_FLOOR_OUTPUT * 12.0
		return if (currentAngle < RESTING_ANGLE) ff * 1.125 - 0.1 else ff + 0.125
	}

	fun calcError(): Double = setpoint - CANcoder.absolutePosition.value

	override fun robotInit() {
		// Report the use of the Kotlin Language for "FRC Usage Report" statistics
		HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)
	}

	override fun robotPeriodic() {}

	override fun autonomousInit() {}

	override fun autonomousPeriodic() {}

	override fun teleopInit() {}

	override fun teleopPeriodic() {
		setpoint = ps4Controller.pov.toDouble()
		setpoint = MathUtil.clamp(setpoint, 0.0, 293.0)

		controlRequestShooterAngle.apply {
			Position = setpoint / 360
			FeedForward = calculateAngleFF(360 * CANcoder.absolutePosition.value)
		}
		if ((minSwitch.get() && (calcError() < 0)) || (maxSwitch.get() && (calcError() > 0)))
			motor.setVoltage(calculateAngleFF(360 * CANcoder.absolutePosition.value))
		else
			motor.setControl(controlRequestShooterAngle)

		SmartDashboard.putNumber("Setpoint", setpoint)
		SmartDashboard.putNumber("Shooter Angle", 360 * CANcoder.absolutePosition.value)
	}

	override fun disabledInit() {}

	override fun disabledPeriodic() {}

	override fun testInit() {}

	override fun testPeriodic() {}

	override fun simulationInit() {}

	override fun simulationPeriodic() {}
}