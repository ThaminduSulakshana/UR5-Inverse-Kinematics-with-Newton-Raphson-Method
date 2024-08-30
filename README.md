# UR5 Inverse Kinematics with Newton-Raphson Method

Inverse kinematics of the UR5 industrial robot using the Newton-Raphson method. The project demonstrates the iterative process of convergence and includes a visualization of the UR5 robot in the CoppeliaSim simulator.

## Project Overview

This project aims to:
- Enhance understanding of the MR code library.
- Increase familiarity with the CoppeliaSim simulator.
- Solidify concepts related to numerical inverse kinematics.

## Contents

- `code/`: Contains the modified `IKinBodyIterates` function and related files.
- `iterates.csv`: A CSV file with the joint vectors for each iteration.
- `log.txt`: A log file showing the output of the function for each iteration.
- `screenshot.png`: A screenshot showing the UR5 robot in the desired end-effector configuration.
- `video.mp4`: A video showing the convergence of the Newton-Raphson process in CoppeliaSim.

## Instructions

1. **Setup**: Ensure you have the MR code library and CoppeliaSim installed.
2. **Run the Code**: Modify the initial guess in the `IKinBodyIterates` function as needed, and run the code to generate the iteration log and CSV file.
3. **Visualize in CoppeliaSim**: Load the generated CSV file into CoppeliaSim to visualize the robot's convergence.
4. **Output**: Review the log, screenshot, and video to verify the accuracy of the solution.

## Usage

- Use the provided `IKinBodyIterates` function to perform the inverse kinematics for the UR5 robot.
- The `iterates.csv` file can be loaded into CoppeliaSim for animation.

## License

[MIT License](License)