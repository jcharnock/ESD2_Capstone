All relevant .m / .blend / .cpp / .hpp / .py files listed below in alphabetical order:

-------------------------------------------------------------------------------------------

analyze_court_restitution.m:
- Computes the coefficient of restitution (COR) from the ball trajectory data. Accepts either a CSV file containing time (t) and XYZ coordinates or direct (t, xyz) input.
- Calculates vertical velocity, detects bounce events from minima in the height signal relative to the world ground height, and estimates COR by comparing the ball's vertical speed immediately before and after each valid impact.
- Outputs the individual COR values, summary statistics such as mean and standard deviation, bounce indices, and relevant plots.


artifactWithNetHeightCheck.m:
- MATLAB simulation/visualization script for testing ball motion with ground-bounce and net-height interaction checks.
- Useful for validating trajectory behavior and court geometry assumptions.
- This is more of a development/side-analysis file than a required part of the main automated tracking pipeline.


blenderLink.m:
- The MATLAB communication helper that sends object pose data to Blender and receives a rendered image back.
- Writes the requested image width, height, object position, and orientation over the TCP connection, sends the object name, then reads the returned RGBA pixel stream from Blender and converts it into a standard MATLAB image array.
- This file is the low-level bridge between MATLAB and the Blender simulation environment, allowing the GUI and dataset-generation scripts to request synchronized camera views programmatically.


blenderServer.py:
- The Blender-side server script that listens for commands from MATLAB, updates an object's position and orientation in the 3D scene, renders the current view, and sends the rendered image data back through a socket connection.
- Sets up Blender's compositor nodes, exposes Start Server and Stop Server controls in a custom Blender panel, and repeatedly handles incoming pose requests using a timer callback.
- This file is the core of the MATLAB-Blender rendering pipeline because it turns Blender into a controllable image server for tennis-ball tracking data.


build_demo_case_library_fixed.m:
- Generates a complete library of predefined demo test cases for the tracking system.
- Defines multiple serve/volley cases, including IN, OUT, and NET examples, creates physically plausible trajectories for each case, and uses the Blender connection to render synchronized images for all camera views by calling 
'generate_case_image_sequence_fixed.m.'
- Saves generated manifests and writes a summary table describing the created cases.
- This file is used to build a reusable dataset so the tracking system can be tested on consistently known examples.


buildWebPayload.m:
- Converts the GUI's internal shot struct into a website payload struct for backend upload
- Extracts metadata, schema version, device/source name, shot category, upload timestamp, shot name, decision, time, valid mask, true XYZ, measured XYZ, GUI-used XYZ, bounce/contact info, selected stereo-pair history, restitution value, and summary counts
- Computes error metrics such as per-sample XYZ error, RMSE values, mean absolute XYZ error, and mean error magnitude when true/measured trajectories are available.
- This file keeps GUI handles, image frames, timers, and other large/non-serializable MATLAB objects out of the website payload.


computeShotErrorStats.m:
- Compares measured 3D tracking results against known ground-truth trajectory data and produces error statistics.
- Computes signed X, Y, Z errors for each valid sample, magnitude error, mean error, maximum error, and RMSE.
- Also displays bounce error if a bounce index is available.
- This file is used to quantify tracking accuracy and helps validate the system against simulated Blender data where the true ball position is already known.


generate_case_image_sequence_fixed.m:
- Renders and saves the full image sequence for one test case, given a case containing time values and true ball coordinates.
- Moves the ball frame by frame in Blender, captures images from both cameras in both stereo pairs, and writes the resulting PNG files to disk.
- Creates a per-case manifest that includes frame timing, image paths, expected classification result (IN, OUT, NET), and true XYZ coordinates.
- The MATLAB server created in the Blender file tennisCourt - FourCams.blend must be active while running this script.


getShotPreset.m:
- Returns predefined motion presets for repeatable serve/volley trajectories.
- Each preset contains the shot name/type, initial position, velocity components, gravity, expected bounce time, and approximate restitution value.
- Current presets include serve_default, serve_t, serve_wide, serve_body, volley_default, volley_short, volley_deep, and volley_cross.
- This script acts as a reusable shot-configuration lookup table and helps standardize serve/volley simulations used throughout the project.


GUI_Professional.m:
- The main field-facing MATLAB GUI intended for the final/professional-use version of the tennis ball tracking system.
- Provides the core user controls, like Render + Track, Clear Trajectory, Track Serve, Track Volley, Compute Restitution, Instant Replay, LED-style decision display, live court plot, image display, and shot readouts.
- Communicates with Blender for four-camera rendering and with the C++ backend for ball detection, stereo-pair selection, and triangulation.
- Focuses on the primary workflow needed for demonstrations or in-field use, while hiding some of the heavier development-only accuracy info
- Stores the latest tracked shot so it can be replayed, classified, plotted, and optionally uploaded to the website/backend.


main.cpp:
- The executable entry point for the triangulation backend.
- Parses command-line arguments, supports a normal processing mode and dummy test mode, loads the backend configuration file, reads the four stereo input images, calls the core processFourViews function, and writes the final tracking result to a text file for MATLAB to read back.
- This file is the wrapper that turns the backend into a command-line tool that the GUI and manifest-based scripts can launch automatically during tracking runs.


Project_GUI_Development_v1.m:
- Development-focused MATLAB GUI for testing, validating, and debugging the tennis ball system.
- Includes the same core pipeline as the professional GUI: Blender rendering control, four-camera image capture, C++ backend calls, serve/volley tracking, instant replay, coefficient of restitution calculation, LED-style outputs, and court/trajectory plotting.
- Adds extra engineering and validation tools such as Run Sweep, Run Validation, Run Weak-Zone Validation, Fit XYZ Correction, Run Camera Accuracy, static validation tables, error maps, error-vs-height plots, failure maps, and additional command-window debugging output.
- Includes website-upload settings and calls uploadLastShotToWebsite.m after selected serve, volley, and COR runs so the latest shot can be sent to a backend/frontend system.
- This file is best treated as the experimental/development GUI, while GUI_Professional.m is the cleaner in-field/demo GUI.


pushShotToWebsite.m:
- Sends a prepared website payload to the backend API as a JSON POST request using MATLAB's webwrite function
- Uses WEB.enabled to decide whether uploading should occur, WEB.apiUrl as the target route, WEB.timeoutSec as the request timeout, and WEB.apiKey as an optional X-API-Key header
- Returns the backend response if the upload succeeds, or returns an early disabled-upload response when web uploading is turned off
- This function should keep the low-level HTTP/webwrite logic separate from the GUI and payload-building code


readServe.m:
- A simple MATLAB utility script for loading and visualizing prerecorded serve trajectory data from a .dat file.
- Reads the file, reorders the coordinate columns into XYZ format, optionally samples the data at a specified interval, and plots the resulting 3D point cloud with scatter3.
- This script is mainly useful for quick inspection of raw serve motion data and should be used for early analysis/visualization rather than as a central part of the final tracking workflow.


rebuild_all_cases_manifest.m:
- Reconstructs a clean master manifest by scanning the image folders directly instead of relying on existing per-case manifest files.
- Searches through the generated serve and volley case directories, identifies synchronized frame sets based on the saved PNG filenames, infers the expected result from the case name, and writes a new all_cases_manifest.csv containing the image paths and frame timing.
- This function is used when earlier manifest files are missing, damaged, or inconsistent because it rebuilds a reliable master index directly from the saved image dataset.


run_camera_position_accuracy.m:
- A camera validation tool that measures how much tracking error is introduced when camera positions and angles are perturbed, such as during simulated weather/wind effects.
- Holds the ball at a fixed 3D location, applies controlled random or grid-based offsets to the stereo camera configurations, reruns tracking for each perturbed setup, and records the resulting XYZ error metrics in a table.
- Can simulate effects such as wind-induced camera motion and produces summary statistics including mean error, maximum error, RMSE, and valid tracking fraction.
- This file is used to evaluate how sensitive the system is to imperfect camera placement and motion.


run_cor_bounce_test.m:
- Generates a multi-bounce tennis ball trajectory for coefficient of restitution testing and can optionally run the tracker on each sample.
- Simulates repeated vertical bounces using a specified ground height, initial position, and approximate rebound factor, stores the true and measured trajectory data, and identifies bounce indices for later restitution analysis.
- Can also plot the bounce path over time and in 3D.
- This file is used to create controlled bounce-test datasets for evaluating how accurately the system can detect rebounds and estimate court restitution behavior.


run_cor_surface_comparison.m:
- Compares bounce behavior across different court surfaces by running the same bounce test with different modeled rebound settings.
- Automatically generates tests for surfaces such as clay, concrete, and grass, calls run_cor_bounce_test.m to create trajectories, then passes the results into analyze_court_restitution.m to compute the measured coefficient of restitution for each surface.
- Returns a summary table containing the modeled rebound value, measured mean COR, standard deviation, and number of valid bounces.
- Useful for demonstrating how the system can distinguish between different court conditions using bounce dynamics.


runShotSweep.m:
- Runs a repeatable batch sweep across multiple named serve and volley presets.
- Accepts a tracking function handle, a list of preset names, a sample interval, and optional per-preset durations.
- Maps user-facing sweep names such as serve_default, serve_t, volley_default, and volley_cross to the internal GUI preset names used by the tracking callbacks.
- Calls the supplied tracking function for each preset, computes tracking error statistics using computeShotErrorStats.m, and returns a results table containing sample count, mean X/Y/Z error, mean magnitude error, maximum error, RMSE, and bounce error.
- This file supports quick comparison of multiple shot types without manually running each serve/volley case one at a time.


tennis_GreenCourt.blend
- The main Blender scene file used to render the tennis environment.
- Contains the tennis court model, ball object, and four-camera setup used for generating synchronized stereo image pairs for tracking and triangulation.
- This file is the visual/geometric foundation of the rendering pipeline because all MATLAB-driven camera captures and object movements are performed inside this Blender scene.
- Note that 'tennis_BlueCourt.blend' and 'tennis_RedCourt.blend' are effectively the same, just with different colored courts


track_case_from_images.m:
- Processes a full test case from a manifest table/CSV and reconstructs the ball trajectory from previously rendered stereo images.
- For each frame, writes a backend configuration file, calls the triangulation executable on the four camera images, and keeps valid measurements.
- It then classifies the shot as IN, OUT, or NET, and produces the coefficient of restitution.


track_case_set_from_manifest.m:
- Runs the ball tracking pipeline across an entire multi-case manifest and summarizes the results.
- Separates the manifest by case_name, calls track_case_from_images.m for each case, stores the returned shot data, and builds a table with the expected result, predicted result, correctness, number of frames tracked, restitution estimate, and bounce location for each case.
- Very useful for batch evaluation because it gives a quick overview of classification accuracy and overall performance across many serve/volley examples.


tracker_backend.cpp:
- Contains the main backend implementation for tennis ball detection, stereo-pair selection, triangulation, configuration parsing, and result-file generation.
- Detects the ball in all four camera views, checks whether each stereo pair is usable, scores competing pairs when both are valid, builds projection matrices from the configured camera geometry, triangulates the 3D world position, verifies/cleans the output, and writes detailed debug information into the returned message string.
- Also includes helper logic for reading the config file and saving the backend result in the key-value format that MATLAB parses.
- This file is the core computational engine of the project and is where the actual image-based tracking and 3D reconstruction happen.


tracker_backend.hpp:
- Shared header file that defines the backend's public interface, data structures, and function declarations.
- Provides the common types used across the backend, such as the configuration structure, per-image detection result, and final stereo output structure.
- Shows the functions that main.cpp calls to load settings, process images, and write result files.
- This file organizes the backend API in one place and keeps the executable entry point and implementation file synchronized through a single shared definition of inputs, outputs, and processing routines.


uploadLastShotToWebsite.m:
- Wrapper helper that takes the GUI's latest shot struct, converts it into a website payload, and sends it to the backend API.
- Calls buildWebPayload.m first to prepare the clean upload struct, then calls pushShotToWebsite.m to perform the JSON POST request.
- Accepts lastShot, shotCategory, and WEB settings, allowing the GUI to upload serve, volley, COR, or other shot types without duplicating website-upload logic inside every callback.
- This file connects the GUI's lastShot output to the website/backend update workflow.
