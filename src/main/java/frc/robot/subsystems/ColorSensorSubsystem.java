package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ColorSensorSubsystem extends PomSubsystem
{
    // Color Sensor
    //------------------------------------------------------------------------------------
    public I2C.Port i2cPort =  I2C.Port.kOnboard;
    public  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    private final ColorMatch m_colorMatcher = new ColorMatch();
    ColorMatchResult match;
    //------------------------------------------------------------------------------------
    public static final Color noteColor = new Color(130, 98, 26);
    public static final Color[] notNoteColors = new Color[]{new Color(54, 113, 86),
    new Color(66, 135, 245),
    new Color(15, 18, 15),
    new Color(100,30,70),
    new Color(255,255,255),
    new Color(0,0,0),
    new Color(100,30,255),
    new Color(189, 183, 170)};



    public ColorSensorSubsystem()
    {
        // adding collors to the dataset of m_colorMatcher
        for(int i = 0;i<notNoteColors.length;i++) m_colorMatcher.addColorMatch(notNoteColors[i]);
        m_colorMatcher.addColorMatch(noteColor);
        setDefaultCommand(this.runOnce(() -> stopMotor()));
    }
    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("is note in", isNoteIn());
        SmartDashboard.putString("color", colorSensor.getColor().toHexString());
    }

    public boolean isNoteIn()
    {
        match = m_colorMatcher.matchClosestColor(colorSensor.getColor());
        return match.color == noteColor;
    } 


}
