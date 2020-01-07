package frc.team832.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.robot.Constants;
import frc.team832.robot.commands.TemplateCommand;

public class Drivetrain extends SubsystemBase {
    private boolean initSuccessful = false;

    private CANTalonFX rightMaster, rightSlave, leftMaster, leftSlave;


   public Drivetrain() {

        rightMaster = new CANTalonFX(Constants.RIGHT_MASTER_CAN_ID);
        leftMaster = new CANTalonFX(Constants.LEFT_MASTER_CAN_ID);
        rightSlave = new CANTalonFX(Constants.RIGHT_SLAVE_CAN_ID);
        leftSlave = new CANTalonFX(Constants.LEFT_SLAVE_CAN_ID);

        setDefaultCommand(new TemplateCommand(this));

        initSuccessful = true;
   }

}