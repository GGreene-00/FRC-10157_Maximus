package frc.robot.subsystems;

import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;

public class LinearServo {
    private final ServoChannel m_channel;
    private final double m_speed; 
    private final double m_length;
    private double setPos = 0.0;
    private double curPos = 0.0;
    private double lastTime;

    public LinearServo(ServoChannel channel, double length, double speed) {
        m_channel = channel;
        m_length = length;
        m_speed = speed;
        m_channel.setPowered(true);
        m_channel.setEnabled(true);
        lastTime = Timer.getFPGATimestamp();
    }

    public void setPosition(double setpoint) {
        setPos = MathUtil.clamp(setpoint, 0.0, m_length);
    }

    public void updateCurPos() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = MathUtil.clamp(currentTime - lastTime, 0.0, 0.02);
        lastTime = currentTime;

    double maxDelta = m_speed * dt;

    if (curPos < setPos - maxDelta) curPos += maxDelta;
        else if (curPos > setPos + maxDelta) curPos -= maxDelta;
        else curPos = setPos;

    double percentExtension = MathUtil.clamp(curPos / m_length, 0.0, 1.0);
        int pulseWidth = (int) MathUtil.clamp(
            1000 + percentExtension * 1000,
            1000,
            2000
        );
        m_channel.setPulseWidth(pulseWidth);
    }

    public double getPosition() { return curPos; }
    public boolean isFinished() { return Math.abs(curPos - setPos) < 0.5; }

    public void setSafePosition(double mm) {
        if (m_length < 30.0) {
            // too small to reach 30mm safely, clamp to max instead
            setPosition(m_length);
        } else {
            setPosition(mm);
        }
    }
}