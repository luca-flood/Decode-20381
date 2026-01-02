# NOTICE
This repository contains the specialized autonomous and control software for Team 20381 during the INTO THE DEEP (2024-2025) competition season.

## Welcome!
This project serves as the codebase for FTC Team 20381. Our software architecture is built for high-precision competition, featuring a global top 3% autonomous routine and advanced control theory implementations.

## Requirements
Android Studio: Ladybug (2024.2) or later.

SDK: FTC Robot Controller SDK v10.2 or later.

Target Hardware: REV Control Hub / Expansion Hub.

## Technical Highlights
Our codebase specializes in high-level robotics systems:

Control Theory: Implementation of Kalman filters, Low/High-pass filters, and PIDF control loops for sensor noise attenuation and state estimation.

Autonomous Pathing: Custom motion profiling using Bézier curve interpolation for smooth, non-linear trajectories.

Computer Vision: Real-time processing via OpenCV, featuring AprilTag 3D pose estimation and color blob detection.

Automation: Deterministic state machines utilizing high-frequency concurrent loops for synchronized subsystem sequences.

## Getting Started
For Team Members

Clone the Repo:

Bash
git clone https://github.com/[Your-Repo-Link]/Team20381-IntoTheDeep.git
Import into Android Studio: Use "Import Project" and select the root folder.

Build: Ensure all Gradle dependencies sync correctly before deploying to the Control Hub.

### For Beginners

If you are new to our stack, we recommend reviewing our implementation of PIDF controllers located in the teamcode/control directory and our OpenCV pipelines in teamcode/vision.

Getting Help
Team Documentation

### Release Information
Version 1.0.0 (Into The Deep)

Feature: Integrated LL + Kinematic for dual phase target tracking.

Feature: Implemented Kalman filter for optimized sensor fusion (IMU + Odometry).

Feature: Deployed OpenCV-based AprilTag localization for sub-centimeter field positioning.

Enhancement: Optimized autonomous routines, currently ranking in the top 5% globally.
