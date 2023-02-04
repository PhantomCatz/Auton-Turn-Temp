package frc.Mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

public class PathSelection {
    private final int TEST_PATH = 1;

    private final SendableChooser<Integer> chosenPath = new SendableChooser<>();

    public void initializePathOptions()
    {
        chosenPath.setDefaultOption(Robot.constants.TEST, 1);
        SmartDashboard.putData(chosenPath);
    }

    public void determinePath()
    {
        if(chosenPath.getSelected() == TEST_PATH)
        {
            Robot.auton.testPath();
        }
    }
}
