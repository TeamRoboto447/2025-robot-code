# Team Roboto 447 - FRC 2025 Reefscape

Welcome to Team Roboto 447's repository for the FIRST Robotics Competition 2025 season: Reefscape!

## About the Game

FRC Reefscape, presented by Haas, is the 2025 FIRST Robotics Competition game. In this underwater-themed challenge, two alliances of three teams each compete to score points through various tasks:

- Harvesting Algae: Robots collect and process algae, placing it in their Net or Processor.
- Seeding Coral: Robots place Coral on their Reef, with higher placements worth more points.
- Ascending to Surface: Robots return to their Barge and hang on either a Shallow or Deep Cage.

The match consists of a 15-second autonomous period followed by a 2-minute and 15-second driver-controlled period. Teams can earn additional ranking points by:

- Completing specific actions during the autonomous period
- Scoring enough Coral on all four Reef levels (or three Reef levels if both teams have scored two algae in their respective processors)
- Scoring enough points by hanging on the Barge
- Winning the match

The game emphasizes both alliance cooperation and inter-alliance collaboration, with a Co-op point system that influences tournament rankings.

## Code Structure

Our robot code is built using Java and the WPILib framework. Here's a basic overview of our code structure:

- `src/main/java/frc/robot/`: Main source code directory
  - `commands/`: Contains command classes for robot actions
  - `subsystems/`: Contains subsystem classes for major robot components
  - `utils/`: Contains utility classes and helper functions
  - `Constants.java`: Stores important constant values
  - `Robot.java`: Main robot class
  - `RobotContainer.java`: Configures and binds commands to inputs

## Getting Started with WPILib

To work on this project, you'll need to install the WPILib development environment:

1. Download the latest WPILib installer from the [WPILib Releases page](https://github.com/wpilibsuite/allwpilib/releases).
2. Choose the appropriate installer for your operating system (Windows, macOS, or Linux).
3. Run the installer and follow the prompts to set up WPILib and Visual Studio Code.

For detailed installation instructions, refer to the [WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).

## Contributing Code

As a student on Team Roboto 447, here's how you should contribute code:

1. **Main Branch**: Only the mentor is allowed to modify the `main` branch directly.
2. **Group-Dev Branch**: This branch is used during shop sessions for collaborative work under mentor guidance.
3. **Personal Branches**: Each student has their own branch named after them. This is where you should do all your individual work.

### Workflow for Students

1. Ensure you're on your personal branch before making any changes.
2. Make your code changes and commit them regularly.
3. Push your changes to your personal branch on GitHub.
4. If you need to incorporate your changes into the main project, notify the mentor for review and integration.

Note: This branch structure is temporary to get us started quickly. We will cover more about Git branching strategies in the future and may adjust our workflow accordingly.

Remember to always communicate with the team about the changes you're making and ask for help if you're unsure about anything!
