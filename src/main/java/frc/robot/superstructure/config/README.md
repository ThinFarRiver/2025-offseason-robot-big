# Configuration-Driven Superstructure Library

This library provides a declarative approach to creating superstructures, eliminating the need for manual boilerplate code and making robot configurations much more readable and maintainable.

## Overview

Instead of manually writing hundreds of lines of state machine code, you can now define your entire superstructure using either:
1. **YAML configuration files** - Perfect for non-programmers and easy modification
2. **Fluent Builder API** - Type-safe programmatic configuration in Java

## Key Benefits

✅ **80% code reduction** - From ~538 lines to ~100 lines  
✅ **Declarative configuration** - Define what you want, not how to do it  
✅ **Automatic validation** - Catches configuration errors at startup  
✅ **Built-in logging** - Comprehensive debugging without manual setup  
✅ **Type safety** - Compile-time validation of configurations  
✅ **Easy maintenance** - Change states/transitions without touching complex code  
✅ **Reusable** - Share configurations across robots and seasons  

## Quick Start

### Option 1: YAML Configuration

1. Create a YAML file defining your superstructure:

```yaml
# superstructure-config.yaml
subsystems:
  elevator:
    type: "elevator"
    parameter_name: "height"
    default_value: 0.0
    min_value: 0.0
    max_value: 2.0

poses:
  idle_pose:
    description: "Robot idle position"
    positions:
      elevator: 0.1
      intake: 10.0

states:
  IDLE:
    pose: "idle_pose"
    description: "Default idle state"

transitions:
  - from: "START"
    to: "IDLE"
    edge_type: "standard"

default_command:
  logic_type: "conditional"
  conditions:
    - condition: "hasCoral"
      state: "CORAL_STOW"
      priority: 1
  fallback_state: "IDLE"
```

2. Load and generate the superstructure:

```java
public class RobotContainer {
    private final GeneratedSuperstructure superstructure;
    
    public RobotContainer() {
        this.superstructure = SuperstructureGenerator
            .fromConfigFile("superstructure-config.yaml")
            .builder()
            .withSubsystem("elevator", elevatorInstance)
            .withSubsystem("intake", intakeInstance)
            .withGamepieceTracker("hasCoral", endEffector::isHasCoral)
            .withCondition("inDangerZone", this::isInDangerZone)
            .build();
    }
}
```

### Option 2: Builder API

```java
var superstructure = SuperstructureBuilder.create()
    .withSubsystem("elevator")
        .ofType("elevator")
        .withParameter("height")
        .withRange(0.0, 2.0)
        .done()
    .withPose("idle_pose")
        .withPosition("elevator", 0.1)
        .withPosition("intake", 10.0)
        .done()
    .withState("IDLE")
        .withPose("idle_pose")
        .withDescription("Default idle state")
        .done()
    .withTransition("START", "IDLE")
        .withEdgeType("standard")
        .done()
    .build()
        .withSubsystemInstance("elevator", elevatorInstance)
        .withGamepieceTracker("hasCoral", endEffector::isHasCoral)
        .generate();
```

## Configuration Reference

### Subsystems

Define the mechanical subsystems that make up your superstructure:

```yaml
subsystems:
  elevator:
    type: "elevator"           # "elevator", "pivot", "roller"
    parameter_name: "height"   # "height", "angle", "voltage"
    default_value: 0.0
    min_value: 0.0            # Optional range validation
    max_value: 2.0
    tunable_prefix: "Elevator" # NetworkTables prefix for tuning
```

### Poses

Define named positions that combine multiple subsystem positions:

```yaml
poses:
  scoring_pose:
    description: "Position for scoring game pieces"
    positions:
      elevator: 1.5     # Elevator height in meters
      intake: 45.0      # Intake angle in degrees
      arm: 90.0         # Arm angle in degrees
```

### States

Define robot states that combine poses with actions (voltages):

```yaml
states:
  SCORING:
    pose: "scoring_pose"
    description: "Ready to score"
    voltages:
      intake_roller: 8.0      # Apply 8V to intake roller
    gamepiece_effects:
      requires: ["hasCoral"]   # State requires having coral
      gives: ["hasAlgae"]      # State gives algae to robot
      removes: ["hasCoral"]    # State removes coral from robot
```

### Transitions

Define allowed state transitions with safety constraints:

```yaml
transitions:
  - from: "STOW"
    to: "SCORING"
    bidirectional: true        # Can go both directions
    restricted: false          # Can be used in path planning
    edge_type: "elevator_first" # Move elevator before other subsystems
    conditions: ["elevatorSafe", "noCollision"]
```

Edge types:
- `standard` - Move all subsystems simultaneously
- `elevator_first` - Move elevator to target position first, then other subsystems
- `sequential` - Move each subsystem one at a time
- `custom` - Use custom command class (specify in `custom_command`)

### Default Command Logic

Define intelligent default behavior based on robot/game state:

```yaml
default_command:
  logic_type: "conditional"
  conditions:
    - condition: "hasCoral"           # If robot has coral
      state: "CORAL_STOW"             # Go to coral stow state
      priority: 1                     # Higher priority = checked first
    - condition: "inDangerZone"
      state: "AVOID"
      priority: 2
  fallback_state: "IDLE"              # Default if no conditions match
```

### Safety Configuration

Define safety constraints and collision avoidance:

```yaml
safety:
  safe_flip_height: 0.5               # Minimum elevator height for arm flipping
  danger_zones:
    - "isInHexagonalReefDangerZone"   # Method names that return boolean
  collision_groups:
    - name: "states_below_flip"
      states: ["GROUND_INTAKE", "STOW"]
      transition_type: "standard"
```

## Usage Examples

### Button Bindings

```java
// Simple state transitions
driverController.a().onTrue(superstructure.runGoal("GROUND_INTAKE"));
driverController.b().onTrue(superstructure.runGoal("SCORING"));
driverController.y().onTrue(superstructure.runGoal("EJECT"));

// Conditional state transitions
driverController.x().onTrue(
    Commands.either(
        superstructure.runGoal("CORAL_STOW"),
        superstructure.runGoal("ALGAE_STOW"),
        () -> superstructure.hasCoral()
    )
);
```

### Autonomous Commands

```java
public Command getAutonomousCommand() {
    return superstructure.runGoal("GROUND_INTAKE")
        .andThen(Commands.waitUntil(superstructure::atGoal))
        .andThen(superstructure.runGoal("STOW"))
        .andThen(Commands.waitUntil(superstructure::atGoal))
        .andThen(superstructure.runGoal("SCORING"))
        .andThen(Commands.waitUntil(superstructure::atGoal))
        .andThen(superstructure.runGoal("EJECT"));
}
```

### Advanced Usage

```java
// Dynamic goal selection
superstructure.runGoal(() -> {
    if (isInScoringZone()) {
        return hasCoral() ? "CORAL_SCORE" : "ALGAE_SCORE";
    }
    return "STOW";
});

// State checking
if (superstructure.getCurrentState().equals("SCORING") && superstructure.atGoal()) {
    // Ready to eject
}

// Gamepiece tracking
boolean hasCoral = superstructure.hasCoral();
boolean hasAlgae = superstructure.hasAlgae();
```

## Migration from Manual Superstructure

### Before (Manual Implementation)
```java
// 538 lines of complex state machine code
public class Superstructure extends SubsystemBase {
    private final Graph<SuperstructureState, EdgeCommand> graph = ...;
    
    public Superstructure(IntakeSubsystem intake, ...) {
        // 50+ lines of graph setup
        for (var state : SuperstructureState.values()) {
            graph.addVertex(state);
        }
        addEdge(SuperstructureState.START, SuperstructureState.IDLE, false, false);
        // ... 30+ more edge definitions
        
        setDefaultCommand(runGoal(() -> {
            // 20+ lines of complex logic
            if (endEffectorArm.isHasCoral()) {
                return SuperstructureState.CORAL_STOW;
            }
            // ... more conditions
        }));
    }
    
    private Command getEdgeCommand(SuperstructureState from, SuperstructureState to) {
        // 100+ lines of command generation logic
        if (from == SuperstructureState.START && to == SuperstructureState.IDLE) {
            return runEndEffectorArm(to.getValue().getPose().endEffectorAngle())
                .alongWith(runIntake(to.getValue().getPose().intakeAngle()))
                .andThen(Commands.waitUntil(endEffectorArm::isAtGoal))
                // ... complex sequencing
        }
        // ... 20+ more edge command definitions
    }
}
```

### After (Configuration-Driven)
```java
// ~50 lines of declarative configuration
var superstructure = SuperstructureBuilder.create()
    .withSubsystem("elevator").ofType("elevator").withRange(0.0, 2.0).done()
    .withSubsystem("intake").ofType("pivot").withRange(-45.0, 180.0).done()
    .withPose("idle").withPosition("elevator", 0.1).withPosition("intake", 10.0).done()
    .withState("IDLE").withPose("idle").withDescription("Default idle state").done()
    .withTransition("START", "IDLE").withEdgeType("standard").done()
    .withDefaultCommand()
        .withCondition("hasCoral", "CORAL_STOW", 1)
        .withFallback("IDLE")
        .done()
    .build()
        .withSubsystemInstance("elevator", elevator)
        .withSubsystemInstance("intake", intake)
        .withGamepieceTracker("hasCoral", endEffector::isHasCoral)
        .generate();
```

## Best Practices

### 1. Start Simple
Begin with basic states and transitions, then add complexity:
```yaml
# Start with this
states:
  IDLE: { pose: "idle_pose" }
  INTAKE: { pose: "intake_pose" }

# Then add details
states:
  INTAKE:
    pose: "intake_pose"
    voltages: { intake_roller: 8.0 }
    gamepiece_effects: { gives: ["hasCoral"] }
```

### 2. Use Descriptive Names
```yaml
# Good
poses:
  coral_ground_intake_position:
    description: "Low position for picking up coral from ground"

# Not ideal  
poses:
  pos1:
    description: "Position 1"
```

### 3. Group Related States
```yaml
# Intake states
CORAL_GROUND_INTAKE: { pose: "ground_intake_pose" }
CORAL_STATION_INTAKE: { pose: "station_intake_pose" }
CORAL_INDEXED_INTAKE: { pose: "indexed_intake_pose" }

# Scoring states
L1_SCORE: { pose: "l1_pose" }
L2_SCORE: { pose: "l2_pose" }
L3_SCORE: { pose: "l3_pose" }
```

### 4. Use Safety-First Transitions
```yaml
transitions:
  # Safe: elevator moves first to avoid collisions
  - from: "GROUND_INTAKE"
    to: "HIGH_SCORE"
    edge_type: "elevator_first"
  
  # Restricted: only use when specifically targeting eject
  - from: "SCORING"
    to: "EJECT"
    restricted: true
```

### 5. Leverage Gamepiece Tracking
```yaml
states:
  CORAL_PICKUP:
    gamepiece_effects: { gives: ["hasCoral"] }
  
  CORAL_SCORE:
    gamepiece_effects: 
      requires: ["hasCoral"]
      removes: ["hasCoral"]

default_command:
  conditions:
    - condition: "hasCoral"
      state: "CORAL_STOW"
      priority: 1
```

## Debugging and Logging

The generated superstructure automatically provides comprehensive logging:

- **State transitions** are logged to NetworkTables
- **Current/goal/next states** are visible in Glass
- **Edge commands** show which transitions are executing
- **Pose tracking** shows actual vs. target positions
- **Gamepiece states** are tracked and logged
- **Configuration validation** catches errors at startup

Access logs in NetworkTables under:
- `GeneratedSuperstructure/State`
- `GeneratedSuperstructure/Goal`
- `GeneratedSuperstructure/AtGoal`
- `GeneratedSuperstructure/EdgeCommand`

## Advanced Features

### Custom Edge Commands
For complex transitions, you can specify custom command classes:

```yaml
transitions:
  - from: "STOW"
    to: "SPECIAL_MANEUVER"
    edge_type: "custom"
    custom_command: "com.team.CustomTransitionCommand"
```

### Dynamic Pose Values
All pose values support tunable numbers for real-time adjustment:

```java
// Values automatically become tunable in NetworkTables
.withPosition("elevator", 1.5)  // Creates "Superstructure/Poses/PoseName/elevator"
```

### Conditional Transitions
Add conditions that must be met for transitions:

```yaml
transitions:
  - from: "GROUND"
    to: "HIGH"
    conditions: ["elevatorSafe", "noCollision"]
```

### State Groups and Safety
Define groups of states with similar safety requirements:

```yaml
safety:
  collision_groups:
    - name: "low_states"
      states: ["GROUND_INTAKE", "STOW"]
      transition_type: "standard"
    - name: "high_states"  
      states: ["HIGH_SCORE", "CLIMB_PREP"]
      transition_type: "elevator_first"
```

## Troubleshooting

### Common Issues

**Configuration not loading:**
- Check YAML syntax (use online validator)
- Verify file path is correct
- Check console for validation errors

**States not reachable:**
- Ensure all states have transition paths to goal
- Check for missing bidirectional transitions
- Review restricted transitions usage

**Subsystems not moving:**
- Verify subsystem instances are registered correctly
- Check that subsystem method names match expected patterns
- Review subsystem type configuration

**Default command not working:**
- Ensure condition suppliers are registered
- Check condition priority ordering
- Verify fallback state exists

### Getting Help

1. **Enable debug logging** - Add `System.out.println()` statements are automatically included
2. **Check NetworkTables** - All state information is logged automatically  
3. **Use Glass** - Visualize state machine execution in real-time
4. **Validate configuration** - Startup validation will catch most issues

## Examples

See `ExampleConfiguredSuperstructure.java` for complete working examples of both YAML and Builder API approaches.

The configuration-driven superstructure library makes robot programming more accessible, maintainable, and reliable. Happy coding! 🤖 