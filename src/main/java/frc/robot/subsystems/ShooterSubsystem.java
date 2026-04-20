package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.servohub.ServoChannel.ChannelId;

// Make sure your LinearServo file is in the same folder, or import it if it's elsewhere!

/**
 * Subsystem that handles shooting mechanism:
 * - Dragger motor
 * - Pull-up motor
 * - Flywheel motors 1 & 2
 * - Hood Linear Servo
 */
public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX dragger;
    private final TalonFX pullUp;
    private final TalonFX flyWheel1;
    private final TalonFX flyWheel2;

    // --- ADD THE SERVO HERE ---
    // Port 3, 70mm max length, assuming roughly 10mm per second speed (tune this speed!)
    private final com.revrobotics.servohub.ServoHub servoHub;
    private final LinearServo hoodServoL;
    private final LinearServo hoodServoR;
    private double hoodTarget = 30.0;
    private static final double HOOD_MAX = 30.0;

    private static final int CAN_ID_DRAGGER = 19;
    private static final int CAN_ID_PULLUP = 32;
    private static final int CAN_ID_FLYWHEEL1 = 33; 
    private static final int CAN_ID_FLYWHEEL2 = 35;

    private static final double DRAGGER_SPEED = 0.45;
    private static final double PULLUP_SPEED = -0.4;
    private static final double FLYWHEEL_SPEED = -0.6;
    private static final double FLYWHEEL_SPEED_SLOW = -0.55;

    private static final double DRAGGER_SPEED_R = -0.15;
    private static final double PULLUP_SPEED_R = 0.3;
    private static final double FLYWHEEL_SPEED_R = 0.4; 

    public ShooterSubsystem() {
        dragger = new TalonFX(CAN_ID_DRAGGER);
        pullUp = new TalonFX(CAN_ID_PULLUP);
        flyWheel1 = new TalonFX(CAN_ID_FLYWHEEL1);
        flyWheel2 = new TalonFX(CAN_ID_FLYWHEEL2);

        // Initialize the REV Servo Hub (Double-check CAN ID 40 in REV Hardware Client!)
        this.servoHub = new com.revrobotics.servohub.ServoHub(40); 
        this.hoodServoL = new LinearServo(
            servoHub.getServoChannel(ChannelId.kChannelId0),
            100.0,
            25.0
        );
        this.hoodServoR = new LinearServo(
            servoHub.getServoChannel(ChannelId.kChannelId1),
            100.0,
            25.0
        );

        // Motor configuration (Don't lose this part!)
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        dragger.getConfigurator().apply(config);
        pullUp.getConfigurator().apply(config);
        flyWheel1.getConfigurator().apply(config);
        flyWheel2.getConfigurator().apply(config);
    }
    // --- CRITICAL: ADD PERIODIC LOOP FOR SERVO MATH ---
    @Override
    public void periodic() {
        hoodServoL.updateCurPos();
        hoodServoR.updateCurPos();
    }

    public void setHoodTarget(double pos) {
        hoodTarget = Math.max(0.0, Math.min(HOOD_MAX, pos));
        hoodServoL.setPosition(hoodTarget);
        hoodServoR.setPosition(hoodTarget);
    }

    public Command setHoodExtendedCommand() {
        return Commands.runOnce(() -> setHoodTarget(30.0), this);
    }

    public Command setHoodRetractedCommand() {
        return Commands.runOnce(() -> setHoodTarget(0.0), this);
    }

    /** Starts flywheel motors immediately */
    public void startFlywheels() {
        flyWheel1.set(-FLYWHEEL_SPEED);
        flyWheel2.set(FLYWHEEL_SPEED);
    }

    public void startFlywheelsSlow() {
        flyWheel1.set(-FLYWHEEL_SPEED_SLOW);
        flyWheel2.set(FLYWHEEL_SPEED_SLOW);
    }

    /** Starts dragger motors */
    public void startDragger() {
        dragger.set(DRAGGER_SPEED);
    }

    /** Starts dragger + pullUp feeder */
    public void startFeeder() {
        dragger.set(DRAGGER_SPEED);
        pullUp.set(PULLUP_SPEED);
    }

    /** Stops the shooter motors */
    public void stopShooter() {
        dragger.set(0);
        pullUp.set(0);
        flyWheel1.set(0);
        flyWheel2.set(0);
    }

    public void feederReverse(){
        dragger.set(DRAGGER_SPEED_R);
        pullUp.set(PULLUP_SPEED_R);
    }

    public void flywheelReverse(){
        flyWheel1.set(-FLYWHEEL_SPEED_R);
        flyWheel2.set(FLYWHEEL_SPEED_R);
    }
    
    public void draggerReverse() {
        dragger.set(DRAGGER_SPEED_R);
    }
    
    public void stopDragger() {
        dragger.set(0);
    }

    // ==========================================
    // COMMAND FACTORIES
    // ==========================================

    /** Returns a command that runs the shooter while held */
    public Command getShootCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> startFlywheels()),
            new WaitCommand(.6),
            Commands.run(() -> startFeeder(), this)
        ).finallyDo(interrupted -> stopShooter());
    }
    public Command getShootCommandSlow() {
        return Commands.sequence(
            Commands.runOnce(() -> startFlywheelsSlow()),
            new WaitCommand(.6),
            Commands.run(() -> startFeeder(), this)
        ).finallyDo(interrupted -> stopShooter());
    }

    
    public Command reverseShootCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> flywheelReverse())
        ).finallyDo(interrupted -> stopShooter());
    }

    // PathPlanner variations
    public Command startShooterCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> startFlywheels()),
            new WaitCommand(.3),
            Commands.run(() -> startFeeder(), this)
        );
    }

    public Command stopShooterCommand() {
        return Commands.runOnce(() -> stopShooter(), this);
    }

    public Command getRDCommand(){
        return Commands.run(() -> draggerReverse(), this)
            .until(() -> false
        ).finallyDo(interrupted -> stopDragger());
    }  
}