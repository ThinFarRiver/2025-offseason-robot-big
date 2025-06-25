package frc.robot.superstructure.config;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.utils.LoggedTracer;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Generated superstructure class that implements the complete state machine
 * based on configuration. This eliminates the need to manually write
 * boilerplate state machine code.
 */
public class GeneratedSuperstructure extends SubsystemBase {
    
    private final Graph<String, EdgeCommand> graph = new DefaultDirectedGraph<>(EdgeCommand.class);
    private final SuperstructureConfig config;
    private final Map<String, SubsystemBase> subsystems;
    private final Map<String, SuperstructureGenerator.GeneratedState> states;
    private final Map<String, SuperstructureGenerator.GeneratedPose> poses;
    private final Map<String, DoubleSupplier> gamepieceTrackers;
    private final Map<String, BooleanSupplier> conditionSuppliers;
    
    @Getter
    private String currentState;
    private String nextState = null;
    @Getter
    private String goalState;
    private EdgeCommand currentEdgeCommand;
    
    public GeneratedSuperstructure(
            SuperstructureConfig config,
            Map<String, SubsystemBase> subsystems,
            Map<String, SuperstructureGenerator.GeneratedState> states,
            Map<String, SuperstructureGenerator.GeneratedPose> poses,
            Map<String, DoubleSupplier> gamepieceTrackers,
            Map<String, BooleanSupplier> conditionSuppliers) {
        
        this.config = config;
        this.subsystems = subsystems;
        this.states = states;
        this.poses = poses;
        this.gamepieceTrackers = gamepieceTrackers;
        this.conditionSuppliers = conditionSuppliers;
        
        // Initialize state to first state in configuration
        this.currentState = states.keySet().iterator().next();
        this.goalState = currentState;
        
        initializeGraph();
        setupDefaultCommand();
        
        System.out.println("Generated superstructure initialized with " + states.size() + " states and " + 
                          config.getTransitions().size() + " transitions.");
    }
    
    private void initializeGraph() {
        // Add all states as vertices
        for (String stateName : states.keySet()) {
            graph.addVertex(stateName);
        }
        
        // Add all transitions as edges
        for (var transition : config.getTransitions()) {
            addEdge(transition);
            
            // Add reverse edge if bidirectional
            if (transition.isBidirectional()) {
                var reverseTransition = SuperstructureConfig.TransitionConfig.builder()
                    .from(transition.getTo())
                    .to(transition.getFrom())
                    .restricted(transition.isRestricted())
                    .edgeType(transition.getEdgeType())
                    .customCommand(transition.getCustomCommand())
                    .conditions(transition.getConditions())
                    .build();
                addEdge(reverseTransition);
            }
        }
    }
    
    private void addEdge(SuperstructureConfig.TransitionConfig transition) {
        EdgeCommand edgeCommand = EdgeCommand.builder()
            .command(generateEdgeCommand(transition))
            .restricted(transition.isRestricted())
            .build();
            
        graph.addEdge(transition.getFrom(), transition.getTo(), edgeCommand);
    }
    
    private Command generateEdgeCommand(SuperstructureConfig.TransitionConfig transition) {
        String from = transition.getFrom();
        String to = transition.getTo();
        String edgeType = transition.getEdgeType() != null ? transition.getEdgeType() : "standard";
        
        System.out.println("Generating edge command: " + from + " -> " + to + " (type: " + edgeType + ")");
        
        switch (edgeType) {
            case "elevator_first":
                return generateElevatorFirstCommand(from, to);
            case "sequential":
                return generateSequentialCommand(from, to);
            case "custom":
                return generateCustomCommand(transition);
            case "standard":
            default:
                return generateStandardCommand(from, to);
        }
    }
    
    private Command generateStandardCommand(String from, String to) {
        var toState = states.get(to);
        
        return runToPose(toState.getPose())
            .andThen(Commands.waitUntil(this::allSubsystemsAtGoal))
            .andThen(runVoltages(toState));
    }
    
    private Command generateElevatorFirstCommand(String from, String to) {
        var toState = states.get(to);
        var pose = toState.getPose();
        
        // Move elevator first, then other subsystems
        Command elevatorCommand = Commands.none();
        Command otherCommands = Commands.none();
        
        for (var subsystemEntry : subsystems.entrySet()) {
            String subsystemName = subsystemEntry.getKey();
            var subsystemConfig = config.getSubsystems().get(subsystemName);
            
            if ("elevator".equals(subsystemConfig.getType())) {
                elevatorCommand = runSubsystemToPosition(subsystemName, pose.getPosition(subsystemName));
            } else {
                Command subsystemCommand = runSubsystemToPosition(subsystemName, pose.getPosition(subsystemName));
                otherCommands = otherCommands.alongWith(subsystemCommand);
            }
        }
        
        return elevatorCommand
            .andThen(Commands.waitUntil(() -> isSubsystemAtGoal("elevator")))
            .andThen(otherCommands)
            .andThen(Commands.waitUntil(this::allSubsystemsAtGoal))
            .andThen(runVoltages(toState));
    }
    
    private Command generateSequentialCommand(String from, String to) {
        var toState = states.get(to);
        var pose = toState.getPose();
        
        Command sequentialCommand = Commands.none();
        
        // Move subsystems one by one in order
        for (var subsystemEntry : subsystems.entrySet()) {
            String subsystemName = subsystemEntry.getKey();
            Command subsystemCommand = runSubsystemToPosition(subsystemName, pose.getPosition(subsystemName));
            
            sequentialCommand = sequentialCommand
                .andThen(subsystemCommand)
                .andThen(Commands.waitUntil(() -> isSubsystemAtGoal(subsystemName)));
        }
        
        return sequentialCommand.andThen(runVoltages(toState));
    }
    
    private Command generateCustomCommand(SuperstructureConfig.TransitionConfig transition) {
        // For now, fall back to standard command
        // In a full implementation, this would use reflection to instantiate custom command classes
        System.out.println("Custom command not implemented, falling back to standard: " + transition.getCustomCommand());
        return generateStandardCommand(transition.getFrom(), transition.getTo());
    }
    
    private Command runToPose(SuperstructureGenerator.GeneratedPose pose) {
        Command combinedCommand = Commands.none();
        
        for (var subsystemEntry : subsystems.entrySet()) {
            String subsystemName = subsystemEntry.getKey();
            DoubleSupplier position = pose.getPosition(subsystemName);
            
            Command subsystemCommand = runSubsystemToPosition(subsystemName, position);
            combinedCommand = combinedCommand.alongWith(subsystemCommand);
        }
        
        return combinedCommand;
    }
    
    private Command runSubsystemToPosition(String subsystemName, DoubleSupplier position) {
        return Commands.runOnce(() -> {
            var subsystem = subsystems.get(subsystemName);
            var subsystemConfig = config.getSubsystems().get(subsystemName);
            
            // Use reflection or type switching to call appropriate methods
            // For now, assume subsystems have standard method names
            try {
                switch (subsystemConfig.getType()) {
                    case "elevator":
                        subsystem.getClass().getMethod("setElevatorPosition", double.class)
                            .invoke(subsystem, position.getAsDouble());
                        break;
                    case "pivot":
                        subsystem.getClass().getMethod("setPivotAngle", double.class)
                            .invoke(subsystem, position.getAsDouble());
                        break;
                    default:
                        System.out.println("Unknown subsystem type: " + subsystemConfig.getType());
                }
            } catch (Exception e) {
                System.err.println("Failed to set position for subsystem " + subsystemName + ": " + e.getMessage());
            }
        });
    }
    
    private Command runVoltages(SuperstructureGenerator.GeneratedState state) {
        return Commands.runOnce(() -> {
            for (var voltageEntry : state.getVoltages().entrySet()) {
                String subsystemName = voltageEntry.getKey();
                double voltage = voltageEntry.getValue().getAsDouble();
                
                var subsystem = subsystems.get(subsystemName);
                
                // Use reflection to call voltage methods
                try {
                    subsystem.getClass().getMethod("setRollerVoltage", double.class)
                        .invoke(subsystem, voltage);
                } catch (Exception e) {
                    System.err.println("Failed to set voltage for subsystem " + subsystemName + ": " + e.getMessage());
                }
            }
        });
    }
    
    private boolean allSubsystemsAtGoal() {
        for (var subsystemEntry : subsystems.entrySet()) {
            if (!isSubsystemAtGoal(subsystemEntry.getKey())) {
                return false;
            }
        }
        return true;
    }
    
    private boolean isSubsystemAtGoal(String subsystemName) {
        var subsystem = subsystems.get(subsystemName);
        
        try {
            return (Boolean) subsystem.getClass().getMethod("isAtGoal").invoke(subsystem);
        } catch (Exception e) {
            System.err.println("Failed to check if subsystem " + subsystemName + " is at goal: " + e.getMessage());
            return true; // Assume at goal if we can't check
        }
    }
    
    private void setupDefaultCommand() {
        if (config.getDefaultCommand() != null) {
            setDefaultCommand(runGoal(this::calculateDefaultGoal));
        }
    }
    
    private String calculateDefaultGoal() {
        var defaultConfig = config.getDefaultCommand();
        if (defaultConfig == null) {
            return currentState;
        }
        
        // Sort conditions by priority
        List<SuperstructureConfig.ConditionalStateConfig> sortedConditions = 
            new ArrayList<>(defaultConfig.getConditions());
        sortedConditions.sort(Comparator.comparingInt(SuperstructureConfig.ConditionalStateConfig::getPriority));
        
        // Check conditions in priority order
        for (var condition : sortedConditions) {
            if (evaluateCondition(condition.getCondition())) {
                return condition.getState();
            }
        }
        
        // Return fallback state if no conditions match
        return defaultConfig.getFallbackState() != null ? defaultConfig.getFallbackState() : currentState;
    }
    
    private boolean evaluateCondition(String conditionName) {
        BooleanSupplier condition = conditionSuppliers.get(conditionName);
        if (condition != null) {
            return condition.getAsBoolean();
        }
        
        System.err.println("Unknown condition: " + conditionName);
        return false;
    }
    
    @Override
    public void periodic() {
        // Handle gamepiece tracking simulation
        if (!RobotBase.isReal() && !RobotConstants.useReplay) {
            handleGamepieceSimulation();
        }
        
        // State machine progression logic (similar to original)
        if (currentEdgeCommand == null || !currentEdgeCommand.getCommand().isScheduled()) {
            if (nextState != null) {
                currentState = nextState;
                nextState = null;
            }
            
            if (!currentState.equals(goalState)) {
                bfs(currentState, goalState).ifPresent(next -> {
                    this.nextState = next;
                    currentEdgeCommand = graph.getEdge(currentState, next);
                    currentEdgeCommand.getCommand().schedule();
                });
            }
        }
        
        // Logging
        Logger.recordOutput("GeneratedSuperstructure/State", currentState);
        Logger.recordOutput("GeneratedSuperstructure/Next", nextState);
        Logger.recordOutput("GeneratedSuperstructure/Goal", goalState);
        if (currentEdgeCommand != null) {
            Logger.recordOutput("GeneratedSuperstructure/EdgeCommand", 
                graph.getEdgeSource(currentEdgeCommand) + " --> " + graph.getEdgeTarget(currentEdgeCommand));
        } else {
            Logger.recordOutput("GeneratedSuperstructure/EdgeCommand", "");
        }
        
        LoggedTracer.record("GeneratedSuperstructure");
    }
    
    private void handleGamepieceSimulation() {
        var currentStateData = states.get(currentState);
        if (currentStateData != null && currentStateData.getGamepieceEffects() != null && atGoal()) {
            var effects = currentStateData.getGamepieceEffects();
            
            // Handle gamepiece gives
            if (effects.getGives() != null) {
                for (String gamepiece : effects.getGives()) {
                    setGamepieceState(gamepiece, true);
                }
            }
            
            // Handle gamepiece removes
            if (effects.getRemoves() != null) {
                for (String gamepiece : effects.getRemoves()) {
                    setGamepieceState(gamepiece, false);
                }
            }
        }
    }
    
    private void setGamepieceState(String gamepiece, boolean hasGamepiece) {
        // This would typically call methods on subsystems to set gamepiece states
        System.out.println("Setting gamepiece " + gamepiece + " to " + hasGamepiece);
    }
    
    @AutoLogOutput(key = "GeneratedSuperstructure/AtGoal")
    public boolean atGoal() {
        return currentState.equals(goalState);
    }
    
    public Command runGoal(String goalState) {
        return runOnce(() -> setGoal(goalState)).andThen(Commands.idle(this));
    }
    
    public Command runGoal(Supplier<String> goalSupplier) {
        return run(() -> setGoal(goalSupplier.get()));
    }
    
    public void setGoal(String goalState) {
        if (this.goalState.equals(goalState)) return;
        this.goalState = goalState;
        // Additional goal setting logic here...
    }
    
    private Optional<String> bfs(String start, String goal) {
        Map<String, String> parents = new HashMap<>();
        Queue<String> queue = new LinkedList<>();
        queue.add(start);
        parents.put(start, null);
        
        while (!queue.isEmpty()) {
            String current = queue.poll();
            if (current.equals(goal)) {
                break;
            }
            
            for (EdgeCommand edge : graph.outgoingEdgesOf(current)) {
                String neighbor = graph.getEdgeTarget(edge);
                if (!parents.containsKey(neighbor)) {
                    parents.put(neighbor, current);
                    queue.add(neighbor);
                }
            }
        }
        
        if (!parents.containsKey(goal)) {
            return Optional.empty();
        }
        
        String nextState = goal;
        while (!nextState.equals(start)) {
            String parent = parents.get(nextState);
            if (parent == null) {
                return Optional.empty();
            } else if (parent.equals(start)) {
                return Optional.of(nextState);
            }
            nextState = parent;
        }
        return Optional.of(nextState);
    }
    
    @Builder(toBuilder = true)
    @Getter
    public static class EdgeCommand extends DefaultEdge {
        private final Command command;
        @Builder.Default
        private final boolean restricted = false;
    }
} 