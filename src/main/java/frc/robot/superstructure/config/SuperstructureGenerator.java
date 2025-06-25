package frc.robot.superstructure.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.superstructure.config.generators.*;
import frc.robot.utils.TunableNumber;
import org.littletonrobotics.junction.Logger;

import java.io.File;
import java.io.IOException;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Main generator class that creates a complete superstructure from configuration.
 * 
 * Usage:
 * 1. Create a YAML configuration file
 * 2. Call SuperstructureGenerator.fromConfig() to generate the superstructure
 * 3. Register subsystems and use the generated superstructure
 */
public class SuperstructureGenerator {
    private static final ObjectMapper YAML_MAPPER = new ObjectMapper(new YAMLFactory());
    
    private final SuperstructureConfig config;
    private final Map<String, SubsystemBase> subsystemInstances;
    private final Map<String, DoubleSupplier> gamepieceTrackers;
    private final Map<String, BooleanSupplier> conditionSuppliers;
    
    private SuperstructureGenerator(SuperstructureConfig config) {
        this.config = config;
        this.subsystemInstances = new HashMap<>();
        this.gamepieceTrackers = new HashMap<>();
        this.conditionSuppliers = new HashMap<>();
    }
    
    /**
     * Creates a SuperstructureGenerator from a YAML configuration file.
     */
    public static SuperstructureGenerator fromConfigFile(String configPath) throws IOException {
        SuperstructureConfig config = YAML_MAPPER.readValue(new File(configPath), SuperstructureConfig.class);
        return new SuperstructureGenerator(config);
    }
    
    /**
     * Creates a SuperstructureGenerator from a configuration object.
     */
    public static SuperstructureGenerator fromConfig(SuperstructureConfig config) {
        return new SuperstructureGenerator(config);
    }
    
    /**
     * Builder class for registering subsystems and conditions before generation.
     */
    public static class Builder {
        private final SuperstructureGenerator generator;
        
        private Builder(SuperstructureGenerator generator) {
            this.generator = generator;
        }
        
        /**
         * Registers a subsystem instance that will be used in the generated superstructure.
         */
        public Builder withSubsystem(String name, SubsystemBase subsystem) {
            generator.subsystemInstances.put(name, subsystem);
            return this;
        }
        
        /**
         * Registers a gamepiece tracker (e.g., hasCoral, hasAlgae).
         */
        public Builder withGamepieceTracker(String name, BooleanSupplier tracker) {
            generator.gamepieceTrackers.put(name, () -> tracker.getAsBoolean() ? 1.0 : 0.0);
            return this;
        }
        
        /**
         * Registers a condition supplier for default command logic.
         */
        public Builder withCondition(String name, BooleanSupplier condition) {
            generator.conditionSuppliers.put(name, condition);
            return this;
        }
        
        /**
         * Generates the complete superstructure.
         */
        public GeneratedSuperstructure build() {
            return generator.generateSuperstructure();
        }
    }
    
    /**
     * Returns a builder for configuring the generator.
     */
    public Builder builder() {
        return new Builder(this);
    }
    
    /**
     * Generates the complete superstructure system.
     */
    private GeneratedSuperstructure generateSuperstructure() {
        System.out.println("Generating superstructure from configuration...");
        
        // Validate configuration
        validateConfiguration();
        
        // Generate poses
        Map<String, GeneratedPose> poses = generatePoses();
        
        // Generate states
        Map<String, GeneratedState> states = generateStates(poses);
        
        // Generate the main superstructure class
        GeneratedSuperstructure superstructure = new GeneratedSuperstructure(
            config, subsystemInstances, states, poses, gamepieceTrackers, conditionSuppliers
        );
        
        System.out.println("Superstructure generation complete!");
        return superstructure;
    }
    
    private void validateConfiguration() {
        // Validate that all referenced poses exist
        for (var stateEntry : config.getStates().entrySet()) {
            String poseName = stateEntry.getValue().getPose();
            if (!config.getPoses().containsKey(poseName)) {
                throw new IllegalArgumentException("State '" + stateEntry.getKey() + "' references unknown pose '" + poseName + "'");
            }
        }
        
        // Validate that all referenced states exist in transitions
        Set<String> stateNames = config.getStates().keySet();
        for (var transition : config.getTransitions()) {
            if (!stateNames.contains(transition.getFrom())) {
                throw new IllegalArgumentException("Transition references unknown state '" + transition.getFrom() + "'");
            }
            if (!stateNames.contains(transition.getTo())) {
                throw new IllegalArgumentException("Transition references unknown state '" + transition.getTo() + "'");
            }
        }
        
        // Validate that all referenced subsystems exist
        for (var subsystemName : config.getSubsystems().keySet()) {
            if (!subsystemInstances.containsKey(subsystemName)) {
                throw new IllegalArgumentException("Configuration references subsystem '" + subsystemName + "' but no instance was registered");
            }
        }
        
        System.out.println("Configuration validation passed.");
    }
    
    private Map<String, GeneratedPose> generatePoses() {
        Map<String, GeneratedPose> poses = new HashMap<>();
        
        for (var poseEntry : config.getPoses().entrySet()) {
            String poseName = poseEntry.getKey();
            var poseConfig = poseEntry.getValue();
            
            Map<String, DoubleSupplier> suppliers = new HashMap<>();
            
            // Create tunable numbers for each position
            for (var positionEntry : poseConfig.getPositions().entrySet()) {
                String subsystemName = positionEntry.getKey();
                double defaultValue = positionEntry.getValue();
                
                // Create tunable number
                String tunableKey = "Superstructure/Poses/" + poseName + "/" + subsystemName;
                TunableNumber tunableNumber = new TunableNumber(tunableKey, defaultValue);
                suppliers.put(subsystemName, tunableNumber::get);
            }
            
            poses.put(poseName, new GeneratedPose(poseName, poseConfig.getDescription(), suppliers));
        }
        
        return poses;
    }
    
    private Map<String, GeneratedState> generateStates(Map<String, GeneratedPose> poses) {
        Map<String, GeneratedState> states = new HashMap<>();
        
        for (var stateEntry : config.getStates().entrySet()) {
            String stateName = stateEntry.getKey();
            var stateConfig = stateEntry.getValue();
            
            // Get the pose
            GeneratedPose pose = poses.get(stateConfig.getPose());
            
            // Create voltage suppliers
            Map<String, DoubleSupplier> voltageSuppliers = new HashMap<>();
            if (stateConfig.getVoltages() != null) {
                for (var voltageEntry : stateConfig.getVoltages().entrySet()) {
                    String subsystemName = voltageEntry.getKey();
                    double voltage = voltageEntry.getValue();
                    
                    // Create tunable number for voltage
                    String tunableKey = "Superstructure/States/" + stateName + "/Voltages/" + subsystemName;
                    TunableNumber tunableNumber = new TunableNumber(tunableKey, voltage);
                    voltageSuppliers.put(subsystemName, tunableNumber::get);
                }
            }
            
            states.put(stateName, new GeneratedState(
                stateName, 
                stateConfig.getDescription(), 
                pose, 
                voltageSuppliers,
                stateConfig.getGamepieceEffects()
            ));
        }
        
        return states;
    }
    
    /**
     * Container class for generated pose data.
     */
    public static class GeneratedPose {
        private final String name;
        private final String description;
        private final Map<String, DoubleSupplier> positions;
        
        public GeneratedPose(String name, String description, Map<String, DoubleSupplier> positions) {
            this.name = name;
            this.description = description;
            this.positions = positions;
        }
        
        public String getName() { return name; }
        public String getDescription() { return description; }
        public Map<String, DoubleSupplier> getPositions() { return positions; }
        
        public DoubleSupplier getPosition(String subsystemName) {
            return positions.getOrDefault(subsystemName, () -> 0.0);
        }
    }
    
    /**
     * Container class for generated state data.
     */
    public static class GeneratedState {
        private final String name;
        private final String description;
        private final GeneratedPose pose;
        private final Map<String, DoubleSupplier> voltages;
        private final SuperstructureConfig.GamepieceEffectsConfig gamepieceEffects;
        
        public GeneratedState(String name, String description, GeneratedPose pose, 
                            Map<String, DoubleSupplier> voltages,
                            SuperstructureConfig.GamepieceEffectsConfig gamepieceEffects) {
            this.name = name;
            this.description = description;
            this.pose = pose;
            this.voltages = voltages;
            this.gamepieceEffects = gamepieceEffects;
        }
        
        public String getName() { return name; }
        public String getDescription() { return description; }
        public GeneratedPose getPose() { return pose; }
        public Map<String, DoubleSupplier> getVoltages() { return voltages; }
        public SuperstructureConfig.GamepieceEffectsConfig getGamepieceEffects() { return gamepieceEffects; }
        
        public DoubleSupplier getVoltage(String subsystemName) {
            return voltages.getOrDefault(subsystemName, () -> 0.0);
        }
    }
} 