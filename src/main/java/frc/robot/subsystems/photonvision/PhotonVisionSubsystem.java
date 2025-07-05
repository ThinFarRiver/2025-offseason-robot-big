package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static frc.robot.RobotConstants.PhotonvisionConstants.SNAPSHOT_ENABLED;
import static frc.robot.RobotConstants.PhotonvisionConstants.SNAPSHOT_PERIOD;

public class PhotonVisionSubsystem extends SubsystemBase {

    private final PhotonVisionIO[] ios;
    private final PhotonVisionIOInputsAutoLogged[] inputs;
    private Timer snapshotTimer = new Timer();

    public PhotonVisionSubsystem(PhotonVisionIO... ios) {
        this.ios = ios;
        inputs = new PhotonVisionIOInputsAutoLogged[ios.length];
        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new PhotonVisionIOInputsAutoLogged();
        }
        snapshotTimer.start();
    }


    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("PhotonVision/Inst" + i, inputs[i]);
        }
        if (snapshotTimer.hasElapsed(SNAPSHOT_PERIOD)) {
            for (int i = 0; i < ios.length; i++) {
                if (SNAPSHOT_ENABLED[i]) ios[i].takeOutputSnapshot();
            }
            snapshotTimer.reset();
        }
    }

    public Optional<Pose2d> getNearestCoralPosition() {
        /*
            Detection logic:
            1. Chose pose with Nearest distance present in last period
            2. Chose pose with Nearest distance and within last 5 periods.
            3. Return Optional.empty()
         */
    }

}
