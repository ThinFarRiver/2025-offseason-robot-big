package frc.robot.superstructure.config;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.*;
import java.util.function.BooleanSupplier;

/**
 * Builder class that provides a fluent API for creating superstructures programmatically.
 * This is useful when you want to define the configuration in code rather than YAML.
 * 
 * Example usage:
 * <pre>{@code
 * var superstructure = SuperstructureBuilder.create()
 *     .withSubsystem("elevator", ElevatorSubsystem.class)
 *         .ofType("elevator")
 *         .withParameter("height")
 *         .withRange(0.0, 2.0)
 *     .withSubsystem("intake", IntakeSubsystem.class)
 *         .ofType("pivot")
 *         .withParameter("angle")
 *         .withRange(-45.0, 180.0)
 *     .withPose("idle")
 *         .withPosition("elevator", 0.1)
 *         .withPosition("intake", 10.0)
 *     .withState("IDLE")
 *         .withPose("idle")
 *         .withDescription("Default idle state")
 *     .withTransition("START", "IDLE")
 *         .withEdgeType("standard")
 *     .withDefaultCommand()
 *         .withCondition("hasCoral", "CORAL_STOW", 1)
 *         .withFallback("IDLE")
 *     .build(elevatorInstance, intakeInstance)
 *         .withGamepieceTracker("hasCoral", () -> endEffector.isHasCoral())
 *         .withCondition("inDangerZone", this::isInDangerZone)
 *         .generate();
 * }</pre>
 */
public class SuperstructureBuilder {
    private final SuperstructureConfig.SuperstructureConfigBuilder configBuilder;
    private final Map<String, SuperstructureConfig.SubsystemConfig.SubsystemConfigBuilder> subsystemBuilders;
    private final Map<String, SuperstructureConfig.PoseConfig.PoseConfigBuilder> poseBuilders;
    private final Map<String, SuperstructureConfig.StateConfig.StateConfigBuilder> stateBuilders;
    private final List<SuperstructureConfig.TransitionConfig.TransitionConfigBuilder> transitionBuilders;
    private SuperstructureConfig.DefaultCommandConfig.DefaultCommandConfigBuilder defaultCommandBuilder;
    private SuperstructureConfig.SafetyConfig.SafetyConfigBuilder safetyBuilder;
    
    private SuperstructureBuilder() {
        this.configBuilder = SuperstructureConfig.builder();
        this.subsystemBuilders = new HashMap<>();
        this.poseBuilders = new HashMap<>();
        this.stateBuilders = new HashMap<>();
        this.transitionBuilders = new ArrayList<>();
    }
    
    public static SuperstructureBuilder create() {
        return new SuperstructureBuilder();
    }
    
    // Subsystem Configuration
    public SubsystemBuilder withSubsystem(String name) {
        var builder = SuperstructureConfig.SubsystemConfig.builder();
        subsystemBuilders.put(name, builder);
        return new SubsystemBuilder(this, builder);
    }
    
    public static class SubsystemBuilder {
        private final SuperstructureBuilder parent;
        private final SuperstructureConfig.SubsystemConfig.SubsystemConfigBuilder builder;
        
        private SubsystemBuilder(SuperstructureBuilder parent, SuperstructureConfig.SubsystemConfig.SubsystemConfigBuilder builder) {
            this.parent = parent;
            this.builder = builder;
        }
        
        public SubsystemBuilder ofType(String type) {
            builder.type(type);
            return this;
        }
        
        public SubsystemBuilder withParameter(String parameterName) {
            builder.parameterName(parameterName);
            return this;
        }
        
        public SubsystemBuilder withDefaultValue(double defaultValue) {
            builder.defaultValue(defaultValue);
            return this;
        }
        
        public SubsystemBuilder withRange(double min, double max) {
            builder.minValue(min).maxValue(max);
            return this;
        }
        
        public SubsystemBuilder withTunablePrefix(String prefix) {
            builder.tunablePrefix(prefix);
            return this;
        }
        
        public SuperstructureBuilder done() {
            return parent;
        }
    }
    
    // Pose Configuration
    public PoseBuilder withPose(String name) {
        var builder = SuperstructureConfig.PoseConfig.builder()
            .positions(new HashMap<>());
        poseBuilders.put(name, builder);
        return new PoseBuilder(this, builder);
    }
    
    public static class PoseBuilder {
        private final SuperstructureBuilder parent;
        private final SuperstructureConfig.PoseConfig.PoseConfigBuilder builder;
        private final Map<String, Double> positions;
        
        private PoseBuilder(SuperstructureBuilder parent, SuperstructureConfig.PoseConfig.PoseConfigBuilder builder) {
            this.parent = parent;
            this.builder = builder;
            this.positions = new HashMap<>();
            builder.positions(positions);
        }
        
        public PoseBuilder withDescription(String description) {
            builder.description(description);
            return this;
        }
        
        public PoseBuilder withPosition(String subsystemName, double position) {
            positions.put(subsystemName, position);
            return this;
        }
        
        public SuperstructureBuilder done() {
            return parent;
        }
    }
    
    // State Configuration
    public StateBuilder withState(String name) {
        var builder = SuperstructureConfig.StateConfig.builder()
            .voltages(new HashMap<>());
        stateBuilders.put(name, builder);
        return new StateBuilder(this, builder);
    }
    
    public static class StateBuilder {
        private final SuperstructureBuilder parent;
        private final SuperstructureConfig.StateConfig.StateConfigBuilder builder;
        private final Map<String, Double> voltages;
        
        private StateBuilder(SuperstructureBuilder parent, SuperstructureConfig.StateConfig.StateConfigBuilder builder) {
            this.parent = parent;
            this.builder = builder;
            this.voltages = new HashMap<>();
            builder.voltages(voltages);
        }
        
        public StateBuilder withPose(String poseName) {
            builder.pose(poseName);
            return this;
        }
        
        public StateBuilder withDescription(String description) {
            builder.description(description);
            return this;
        }
        
        public StateBuilder withVoltage(String subsystemName, double voltage) {
            voltages.put(subsystemName, voltage);
            return this;
        }
        
        public StateBuilder withGamepieceEffects(SuperstructureConfig.GamepieceEffectsConfig effects) {
            builder.gamepieceEffects(effects);
            return this;
        }
        
        public StateBuilder givesGamepiece(String... gamepieces) {
            var effects = builder.build().getGamepieceEffects();
            if (effects == null) {
                effects = SuperstructureConfig.GamepieceEffectsConfig.builder()
                    .gives(new ArrayList<>())
                    .build();
            }
            
            var newGives = new ArrayList<>(effects.getGives() != null ? effects.getGives() : List.of());
            newGives.addAll(Arrays.asList(gamepieces));
            
            builder.gamepieceEffects(effects.toBuilder().gives(newGives).build());
            return this;
        }
        
        public StateBuilder removesGamepiece(String... gamepieces) {
            var effects = builder.build().getGamepieceEffects();
            if (effects == null) {
                effects = SuperstructureConfig.GamepieceEffectsConfig.builder()
                    .removes(new ArrayList<>())
                    .build();
            }
            
            var newRemoves = new ArrayList<>(effects.getRemoves() != null ? effects.getRemoves() : List.of());
            newRemoves.addAll(Arrays.asList(gamepieces));
            
            builder.gamepieceEffects(effects.toBuilder().removes(newRemoves).build());
            return this;
        }
        
        public StateBuilder requiresGamepiece(String... gamepieces) {
            var effects = builder.build().getGamepieceEffects();
            if (effects == null) {
                effects = SuperstructureConfig.GamepieceEffectsConfig.builder()
                    .requires(new ArrayList<>())
                    .build();
            }
            
            var newRequires = new ArrayList<>(effects.getRequires() != null ? effects.getRequires() : List.of());
            newRequires.addAll(Arrays.asList(gamepieces));
            
            builder.gamepieceEffects(effects.toBuilder().requires(newRequires).build());
            return this;
        }
        
        public SuperstructureBuilder done() {
            return parent;
        }
    }
    
    // Transition Configuration
    public TransitionBuilder withTransition(String from, String to) {
        var builder = SuperstructureConfig.TransitionConfig.builder()
            .from(from)
            .to(to);
        transitionBuilders.add(builder);
        return new TransitionBuilder(this, builder);
    }
    
    public static class TransitionBuilder {
        private final SuperstructureBuilder parent;
        private final SuperstructureConfig.TransitionConfig.TransitionConfigBuilder builder;
        
        private TransitionBuilder(SuperstructureBuilder parent, SuperstructureConfig.TransitionConfig.TransitionConfigBuilder builder) {
            this.parent = parent;
            this.builder = builder;
        }
        
        public TransitionBuilder bidirectional() {
            builder.bidirectional(true);
            return this;
        }
        
        public TransitionBuilder restricted() {
            builder.restricted(true);
            return this;
        }
        
        public TransitionBuilder withEdgeType(String edgeType) {
            builder.edgeType(edgeType);
            return this;
        }
        
        public TransitionBuilder withCustomCommand(String customCommand) {
            builder.customCommand(customCommand);
            return this;
        }
        
        public TransitionBuilder withConditions(String... conditions) {
            builder.conditions(Arrays.asList(conditions));
            return this;
        }
        
        public SuperstructureBuilder done() {
            return parent;
        }
    }
    
    // Default Command Configuration
    public DefaultCommandBuilder withDefaultCommand() {
        defaultCommandBuilder = SuperstructureConfig.DefaultCommandConfig.builder()
            .conditions(new ArrayList<>());
        return new DefaultCommandBuilder(this, defaultCommandBuilder);
    }
    
    public static class DefaultCommandBuilder {
        private final SuperstructureBuilder parent;
        private final SuperstructureConfig.DefaultCommandConfig.DefaultCommandConfigBuilder builder;
        private final List<SuperstructureConfig.ConditionalStateConfig> conditions;
        
        private DefaultCommandBuilder(SuperstructureBuilder parent, SuperstructureConfig.DefaultCommandConfig.DefaultCommandConfigBuilder builder) {
            this.parent = parent;
            this.builder = builder;
            this.conditions = new ArrayList<>();
            builder.conditions(conditions);
        }
        
        public DefaultCommandBuilder withLogicType(String logicType) {
            builder.logicType(logicType);
            return this;
        }
        
        public DefaultCommandBuilder withCondition(String condition, String state, int priority) {
            conditions.add(SuperstructureConfig.ConditionalStateConfig.builder()
                .condition(condition)
                .state(state)
                .priority(priority)
                .build());
            return this;
        }
        
        public DefaultCommandBuilder withFallback(String fallbackState) {
            builder.fallbackState(fallbackState);
            return this;
        }
        
        public SuperstructureBuilder done() {
            return parent;
        }
    }
    
    // Safety Configuration
    public SafetyBuilder withSafety() {
        safetyBuilder = SuperstructureConfig.SafetyConfig.builder();
        return new SafetyBuilder(this, safetyBuilder);
    }
    
    public static class SafetyBuilder {
        private final SuperstructureBuilder parent;
        private final SuperstructureConfig.SafetyConfig.SafetyConfigBuilder builder;
        
        private SafetyBuilder(SuperstructureBuilder parent, SuperstructureConfig.SafetyConfig.SafetyConfigBuilder builder) {
            this.parent = parent;
            this.builder = builder;
        }
        
        public SafetyBuilder withSafeFlipHeight(double height) {
            builder.safeFlipHeight(height);
            return this;
        }
        
        public SafetyBuilder withDangerZones(String... dangerZones) {
            builder.dangerZones(Arrays.asList(dangerZones));
            return this;
        }
        
        public SuperstructureBuilder done() {
            return parent;
        }
    }
    
    // Build the configuration
    public ConfiguredSuperstructure build() {
        // Build all subsystem configs
        Map<String, SuperstructureConfig.SubsystemConfig> subsystems = new HashMap<>();
        for (var entry : subsystemBuilders.entrySet()) {
            subsystems.put(entry.getKey(), entry.getValue().build());
        }
        
        // Build all pose configs
        Map<String, SuperstructureConfig.PoseConfig> poses = new HashMap<>();
        for (var entry : poseBuilders.entrySet()) {
            poses.put(entry.getKey(), entry.getValue().build());
        }
        
        // Build all state configs
        Map<String, SuperstructureConfig.StateConfig> states = new HashMap<>();
        for (var entry : stateBuilders.entrySet()) {
            states.put(entry.getKey(), entry.getValue().build());
        }
        
        // Build all transition configs
        List<SuperstructureConfig.TransitionConfig> transitions = new ArrayList<>();
        for (var builder : transitionBuilders) {
            transitions.add(builder.build());
        }
        
        // Build the complete configuration
        SuperstructureConfig config = configBuilder
            .subsystems(subsystems)
            .poses(poses)
            .states(states)
            .transitions(transitions)
            .defaultCommand(defaultCommandBuilder != null ? defaultCommandBuilder.build() : null)
            .safety(safetyBuilder != null ? safetyBuilder.build() : null)
            .build();
        
        return new ConfiguredSuperstructure(config);
    }
    
    /**
     * Intermediate class that holds the configuration and allows
     * registration of subsystem instances and conditions.
     */
    public static class ConfiguredSuperstructure {
        private final SuperstructureConfig config;
        private final Map<String, SubsystemBase> subsystemInstances = new HashMap<>();
        private final Map<String, BooleanSupplier> gamepieceTrackers = new HashMap<>();
        private final Map<String, BooleanSupplier> conditionSuppliers = new HashMap<>();
        
        private ConfiguredSuperstructure(SuperstructureConfig config) {
            this.config = config;
        }
        
        public ConfiguredSuperstructure withSubsystemInstance(String name, SubsystemBase subsystem) {
            subsystemInstances.put(name, subsystem);
            return this;
        }
        
        public ConfiguredSuperstructure withGamepieceTracker(String name, BooleanSupplier tracker) {
            gamepieceTrackers.put(name, tracker);
            return this;
        }
        
        public ConfiguredSuperstructure withCondition(String name, BooleanSupplier condition) {
            conditionSuppliers.put(name, condition);
            return this;
        }
        
        public GeneratedSuperstructure generate() {
            // Convert boolean suppliers to double suppliers for gamepiece trackers
            Map<String, java.util.function.DoubleSupplier> doubleGamepieceTrackers = new HashMap<>();
            for (var entry : gamepieceTrackers.entrySet()) {
                doubleGamepieceTrackers.put(entry.getKey(), () -> entry.getValue().getAsBoolean() ? 1.0 : 0.0);
            }
            
            return SuperstructureGenerator.fromConfig(config)
                .builder()
                .build();
        }
    }
} 