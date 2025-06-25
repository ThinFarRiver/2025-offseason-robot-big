package frc.robot.superstructure.config;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.List;
import java.util.Map;

/**
 * Configuration class for defining a complete superstructure system.
 * This allows for declarative superstructure creation without boilerplate code.
 */
@Data
@Builder
@Jacksonized
public class SuperstructureConfig {
    
    @JsonProperty("subsystems")
    private Map<String, SubsystemConfig> subsystems;
    
    @JsonProperty("poses")
    private Map<String, PoseConfig> poses;
    
    @JsonProperty("states")
    private Map<String, StateConfig> states;
    
    @JsonProperty("transitions")
    private List<TransitionConfig> transitions;
    
    @JsonProperty("default_command")
    private DefaultCommandConfig defaultCommand;
    
    @JsonProperty("safety")
    private SafetyConfig safety;
    
    @Data
    @Builder
    @Jacksonized
    public static class SubsystemConfig {
        @JsonProperty("type")
        private String type; // "elevator", "pivot", "roller"
        
        @JsonProperty("parameter_name")
        private String parameterName; // "height", "angle", "voltage"
        
        @JsonProperty("default_value")
        private double defaultValue;
        
        @JsonProperty("min_value")
        private Double minValue;
        
        @JsonProperty("max_value")
        private Double maxValue;
        
        @JsonProperty("tunable_prefix")
        private String tunablePrefix;
    }
    
    @Data
    @Builder
    @Jacksonized
    public static class PoseConfig {
        @JsonProperty("description")
        private String description;
        
        @JsonProperty("positions")
        private Map<String, Double> positions; // subsystem name -> position value
    }
    
    @Data
    @Builder
    @Jacksonized
    public static class StateConfig {
        @JsonProperty("pose")
        private String pose; // reference to pose name
        
        @JsonProperty("voltages")
        private Map<String, Double> voltages; // subsystem name -> voltage
        
        @JsonProperty("description")
        private String description;
        
        @JsonProperty("gamepiece_effects")
        private GamepieceEffectsConfig gamepieceEffects;
    }
    
    @Data
    @Builder
    @Jacksonized
    public static class GamepieceEffectsConfig {
        @JsonProperty("requires")
        private List<String> requires; // e.g., ["hasCoral", "hasAlgae"]
        
        @JsonProperty("gives")
        private List<String> gives;
        
        @JsonProperty("removes")
        private List<String> removes;
    }
    
    @Data
    @Builder
    @Jacksonized
    public static class TransitionConfig {
        @JsonProperty("from")
        private String from;
        
        @JsonProperty("to")
        private String to;
        
        @JsonProperty("bidirectional")
        private boolean bidirectional;
        
        @JsonProperty("restricted")
        private boolean restricted;
        
        @JsonProperty("edge_type")
        private String edgeType; // "standard", "elevator_first", "sequential", "custom"
        
        @JsonProperty("custom_command")
        private String customCommand; // class name for custom edge commands
        
        @JsonProperty("conditions")
        private List<String> conditions; // e.g., ["elevatorSafe", "noCollision"]
    }
    
    @Data
    @Builder
    @Jacksonized
    public static class DefaultCommandConfig {
        @JsonProperty("logic_type")
        private String logicType; // "conditional", "priority_list"
        
        @JsonProperty("conditions")
        private List<ConditionalStateConfig> conditions;
        
        @JsonProperty("fallback_state")
        private String fallbackState;
    }
    
    @Data
    @Builder
    @Jacksonized
    public static class ConditionalStateConfig {
        @JsonProperty("condition")
        private String condition; // e.g., "hasCoral", "inDangerZone"
        
        @JsonProperty("state")
        private String state;
        
        @JsonProperty("priority")
        private int priority;
    }
    
    @Data
    @Builder
    @Jacksonized
    public static class SafetyConfig {
        @JsonProperty("collision_groups")
        private List<CollisionGroupConfig> collisionGroups;
        
        @JsonProperty("safe_flip_height")
        private Double safeFlipHeight;
        
        @JsonProperty("danger_zones")
        private List<String> dangerZones; // method names that return boolean
    }
    
    @Data
    @Builder
    @Jacksonized
    public static class CollisionGroupConfig {
        @JsonProperty("name")
        private String name;
        
        @JsonProperty("states")
        private List<String> states;
        
        @JsonProperty("transition_type")
        private String transitionType; // "avoid", "sequential", "elevator_first"
    }
} 