package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChainSubsystem extends SubsystemBase {

    private final TalonFX chainMotor;
    private static final int CAN_ID_CHAIN = 34;

    private static final double CHAIN_FORWARD = 0.2;
    private static final double CHAIN_REVERSE = -0.15;
    private static final double CHAIN_FORWARD_SLOW = 0.1;

    private static final double CURRENT_LIMIT = 40.0; 
    private static final double VELOCITY_THRESHOLD = 1.0;
    private static final double STALL_DELAY = 0.1;
    private static final double DEFAULT_CHAIN_TIME = 1.0;

    private final Timer stallTimer = new Timer();
    private boolean stallDetected = false;

    public ChainSubsystem() {
        chainMotor = new TalonFX(CAN_ID_CHAIN);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        chainMotor.getConfigurator().apply(config);
    }

    // Stall-protected motor control
    private void runWithProtection(double speed) {
        double current = chainMotor.getStatorCurrent().getValueAsDouble();
        double velocity = Math.abs(chainMotor.getVelocity().getValueAsDouble());

        boolean possibleStall = current > CURRENT_LIMIT && velocity < VELOCITY_THRESHOLD;

        if (possibleStall) {
            if (!stallDetected) {
                stallTimer.reset();
                stallTimer.start();
                stallDetected = true;
            }
            if (stallTimer.hasElapsed(STALL_DELAY)) {
                chainMotor.set(0);
                return;
            }
        } else {
            stallDetected = false;
            stallTimer.stop();
            stallTimer.reset();
        }

        chainMotor.set(speed);
    }

    public void moveForward() {
        runWithProtection(CHAIN_FORWARD); 
    }
    public void moveForwardSlow() {
        runWithProtection(CHAIN_FORWARD_SLOW);
    }

    public void moveReverse() {
        runWithProtection(CHAIN_REVERSE);
    }

    public void stop() {
        chainMotor.set(0);
        stallDetected = false;
        stallTimer.stop();
        stallTimer.reset();
    }

    // Instant control commands
    public Command intakeDrop() {
        return Commands.startEnd(
            () -> moveForward(),
            () -> stop(),
            this
        ).withTimeout(1.0);
    }

    public Command intakeLift() {
        return Commands.startEnd(
            () -> moveReverse(),
            () -> stop(),
            this
        ).withTimeout(1.0);
    }

    public Command intakeJostle() {
        return Commands.sequence(
            // Lift for 0.5 seconds
            Commands.run(() -> moveReverse(), this)
                .withTimeout(0.75),

            // Immediately drop for 0.4 seconds
            Commands.run(() -> moveForwardSlow(), this)
                .withTimeout(0.65)
        ).finallyDo(interrupted -> stop());
    }

        public Command intakeStop() {
        return Commands.run(() -> {
            stop();
        }, this
            ).finallyDo(interrupted -> stop());
    }

    // Timed chain command — applies protection first, then stops after default time
    public Command runForDefaultTime(double speed) {
        return Commands.sequence(
            Commands.run(() -> runWithProtection(speed), this),
            new WaitCommand(DEFAULT_CHAIN_TIME),
            Commands.runOnce(this::stop, this)
        );
    }
}
