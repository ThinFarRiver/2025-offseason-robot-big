package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import org.frcteam6941.looper.Updatable;

public class GamepieceTracker implements Updatable {
    //Todo: Consider moving this class to superstructure
    public static GamepieceTracker instance;
    //Current Gamepiece tracking
    @Getter
    private boolean endeffectorHasCoral = false;
    @Getter
    private boolean endeffectorHasAlgae = false;
    @Getter
    private boolean intakeHasCoral = false;

    public static GamepieceTracker getInstance() {
        if (instance == null) {
            instance = new GamepieceTracker();
        }
        return instance;
    }

    /**
     * Sets whether coral is detected EE
     *
     * @param endeffectorHasCoral true if coral is detected, false otherwise
     */
    public void setEndeffectorHasCoral(boolean endeffectorHasCoral) {
        this.endeffectorHasCoral = endeffectorHasCoral;
        SmartDashboard.putBoolean("GamePiece/EEHasCoral", endeffectorHasCoral);
    }

    /**
     * Sets whether algae is detected EE
     *
     * @param endeffectorHasAlgae true if algae is detected, false otherwise
     */
    public void setEndeffectorHasAlgae(boolean endeffectorHasAlgae) {
        this.endeffectorHasAlgae = endeffectorHasAlgae;
        SmartDashboard.putBoolean("GamePiece/EEHasAlgae", endeffectorHasAlgae);
    }

    /**
     * Sets whether coral is detected intake
     *
     * @param intakeHasCoral true if coral is detected, false otherwise
     */
    public void setintakeHasCoral(boolean intakeHasCoral) {
        this.intakeHasCoral = intakeHasCoral;
        SmartDashboard.putBoolean("GamePiece/IntakeHasCoral", intakeHasCoral);
    }


}
