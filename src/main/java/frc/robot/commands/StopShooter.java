package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class StopShooter extends Command {
    private Shooter s_Shooter;
    private Indexer s_Indexer;

    public StopShooter(Shooter s_Shooter, Indexer s_Indexer){
        this.s_Indexer = s_Indexer; 
        this.s_Shooter = s_Shooter;
        addRequirements(s_Shooter, s_Indexer);
    }
    public void execute(){
        //Set the shooter to the desired speed
        s_Shooter.stop();
        s_Indexer.stop(); 
    }
    public void isInterrupted(){
        //Stop shooter if command is interrupted
        s_Shooter.stop();
    }
    public void end(){
        //stop shooter when command ends
        s_Shooter.stop();
    }
    public boolean isFinished(){
        return true;
    }
}
