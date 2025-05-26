package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Builder(toBuilder = true, access = AccessLevel.PACKAGE)
@Getter
public class SuperstructureStateData {
    @Builder.Default
    private final SuperstructurePose pose = new SuperstructurePose(() -> 0.0, () -> 0.0, () -> 0.0);

    @Builder.Default private final DoubleSupplier intakeVolts = () -> 0.0;
    @Builder.Default private final DoubleSupplier endEffectorVolts = () -> 0.0;
} 