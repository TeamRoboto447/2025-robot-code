# FRC Robot Code Design: Architecture and Best Practices

## Overview

This document outlines the design principles and usage guidelines for our FRC robot code. It covers subsystems, commands, utility functions, constants, and overall project structure to ensure consistent, maintainable, and efficient code.

## Key Concepts and Terminology

Before diving into the specifics, here are some key concepts and terms used throughout this document:

- **API (Application Programming Interface)**: A set of defined methods that allow different parts of the code to communicate with each other.
- **Static Method**: A method that belongs to a class rather than an instance of the class. It can be called without creating an object of the class.
- **Instance**: A specific occurrence of a class, also known as an object.
- **Constructor**: A special method used to initialize objects. It's called when an object of a class is created.
- **Instantiation**: The process of creating an instance (object) of a class.
- **Extend**: In Java, this means a class is inheriting properties and methods from another class.
- **Override**: Redefining a method in a subclass that is already defined in its superclass.
- **Subsystem**: A major component of the robot, like the drivetrain or an arm mechanism.
- **Command**: A specific action or behavior that the robot can perform.
- **Periodic**: A method that runs repeatedly at a fixed time interval.

## Project Structure

Our project follows a standard WPILib command-based structure, with a clear organizational hierarchy:

- `src/main/java/frc/robot/`
  - `commands/`: Contains all command classes
  - `subsystems/`: Contains all subsystem classes
  - `utils/`: Contains utility classes and helper functions
  - `Constants.java`: Stores all constant values
  - `Robot.java`: Main robot class, This file is not to be touched
  - `RobotContainer.java`: Binds commands to inputs and initializes subsystems

## Subsystems

### Design Principles

1. **Hardware Abstraction**: Directly control motors, sensors, and physical components.
2. **State Management**: Track current and desired subsystem states.
3. **Safety Features**: Implement safeguards like limit switches and current monitoring.
4. **API Design**: Provide clear, high-level methods for commands to use.

### Implementation Guidelines

- Extend the `SubsystemBase` class
- Initialize hardware components in the constructor
- Implement `periodic()` method for continuous updates and safety checks
- Provide public methods that offer a clean API for commands
- Keep hardware-specific details private within the subsystem

## Commands

### Design Principles

1. **Single Responsibility**: Perform one specific action
2. **Subsystem Interaction**: Use subsystem APIs, never directly control hardware
3. **Input Handling**: Process inputs from operators, sensors, or other sources
4. **State Transitions**: Clearly define command start, end, and interruption conditions

### Implementation Guidelines

- Extend the `CommandBase` class
- Declare required subsystems in the constructor using `addRequirements()`
- Implement `initialize()`, `execute()`, `end()`, and `isFinished()` methods as needed
- Use `execute()` for continuous actions and checks
- Return `true` in `isFinished()` when the command's goal is achieved

## Constants

### Guidelines

1. Store all configuration values in `Constants.java`
2. Organize constants into nested classes based on subsystem or functionality
3. Use UPPER_SNAKE_CASE for all constant values
4. Provide clear, descriptive names

## Utility Functions

### Guidelines

1. Create separate utility classes for distinct functionality
2. Make utility methods static for easy access
3. Provide clear documentation for each method
4. Keep utility functions focused and single-purpose
5. Prevent instantiation of utility classes

## RobotContainer

### Responsibilities

1. Initialize all subsystems
2. Configure default commands for subsystems
3. Set up button bindings for operator interface
4. Create methods for autonomous command selection

## Best Practices

1. **Separation of Concerns**: Maintain clear boundaries between subsystems, commands, and utilities
2. **Reusability**: Design components to be as flexible as possible
3. **Testability**: Structure code to support unit testing
4. **Documentation**: Thoroughly comment complex logic
5. **Consistent Naming**: Use clear, consistent naming conventions
6. **Version Control**: Make frequent, small, well-described commits
7. **Code Review**: Implement peer review for all code changes

## Workflow

1. Identify robot functionalities
2. Create corresponding subsystems
3. Implement subsystem methods
4. Develop commands using subsystem APIs
5. Create utility functions for common operations
6. Configure `RobotContainer`
7. Comprehensive testing

By following these guidelines, we ensure a modular, maintainable, and efficient robot code structure.
