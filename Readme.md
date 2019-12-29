ENAE 788M PRG Husky Obstacle Course Program

This code set operates on the PRG Husky to autonomously 


statemachine.py - this is the state machine which schedules all other subscripts and moduled.  Calling this script will begin the autonomous navigation of the ENAE 788M obstacle course.

final.launch - this launch file launches all drivers on the bebop required for flight.  Front camera, Bebop driver, and bottom camera.

MoveToController_v2.py - this is the waypoint controller for the quad.  It takes in waypoint quaternions and controlls position and yaw based on a PID system.  This must be running for waypoint navigation.

Window_and_Wall.py - This function crosses the first wall and goes through the window.  It uses a GMM to identify the window and PnP to estimate position from it before shooting through.

Bringe.py - This function seeks out the bridge obstacle and controls the quad to line up and cross over the bridge.

circles_detect_land.py - this script seeks out circular or square bullseye tags and controls the quad to land on the target.

circles_detect_land_yaw.py - this script seeks out circular or square bullseye tags and controls the quad to yaw over the target.  This method was required to prevent excessive drift during yaw manuevers.

FindFeet_Controller_V2.py - this script controlls the drone to find wooden feet of the wall obstacles and navigate through the wall obstacle.

