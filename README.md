# 3D BREP Software System User Manual

**Author**: Aditya Bali  
**Date**: October 30, 2024

## Table of Contents

1. [Overview](#overview)
2. [Getting Started](#getting-started)  
   2.1 [System Requirements](#system-requirements)  
   2.2 [Compiling the Software](#compiling-the-software)  
      2.2.1 [Using Makefile](#using-makefile)
3. [Using the Software](#using-the-software)  
   3.1 [Launching the Application](#launching-the-application)  
   3.2 [Command-Line Interface (CLI)](#command-line-interface-cli)  
      3.2.1 [Available Commands](#available-commands)  
      3.2.2 [Example Usage](#example-usage)
4. [Sample Inputs and Outputs](#sample-inputs-and-outputs)  
   4.1 [Test Case 1: Model with Holes](#test-case-1-model-with-holes)  
   4.2 [Test Case 2: Simple Cube](#test-case-2-simple-cube)  
   4.3 [Test Case 3: Non-Closed Loop](#test-case-3-non-closed-loop)
5. [Known Limitations](#known-limitations)

## Overview

Welcome to the 3D BREP (Boundary Representation) Software System User Manual. This software is designed to facilitate the creation, manipulation, validation, and visualization of 3D models using BREP methodologies. Whether you are an engineer, designer, or enthusiast, this tool provides essential functionalities such as loading models from JSON files, performing geometric transformations, projecting models, executing cross-section operations, and managing undo/redo actions. The system is implemented in C++ and utilizes the [nlohmann/json library](https://github.com/nlohmann/json) for seamless JSON parsing.

## Getting Started

### System Requirements

- **Operating System**: Windows, Linux, or macOS.
- **Compiler**: GCC or compatible C++ compiler supporting C++11.
- **Python**: Python 3.x installed for rendering functionalities.
- **Libraries**: `nlohmann/json` library for JSON parsing.

### Compiling the Software

#### Using Makefile

1. Open your terminal or command prompt.
2. Navigate to the project root directory (`3D Software/`).
3. Run the following command to compile the project:

   ```bash
   make
   ```

4. Upon successful compilation, the executable `3d_software.exe` will be available in the `bin/` directory.

## Using the Software

### Launching the Application

1. Open your terminal or command prompt.
2. Navigate to the `bin/` directory:

   ```bash
   cd bin
   ```

3. Execute the application:

   ```bash
   ./3d_software.exe
   ```

### Command-Line Interface (CLI)

The software operates through a command-line interface, allowing users to input commands to perform various operations on BREP models.

#### Available Commands

- `import <JSON_FILE>`: Load a BREP model from a JSON file.
- `save <JSON_FILE>`: Save the current BREP model to a JSON file.
- `rotate <axis> <angle>`: Rotate the model around the specified axis (X, Y, Z) by a given angle.
- `translate <axis> <units>`: Translate the model along the specified axis (X, Y, Z) by a given number of units.
- `project <view>`: Project the model orthogonally onto a specified view (Top, Bottom, Front, Back, Left, Right).
- `cross_section <plane>`: Perform a cross-section operation with the specified plane.
- `undo`: Revert the last operation.
- `redo`: Reapply the last undone operation.
- `render <JSON_FILE>`: Generate a visual representation of the model.
- `help`: Display the list of available commands.
- `exit`: Close the application.

#### Example Usage

- **Importing a Model**:

  ```console
  import HoleTest.json
  ```

- **Rotating the Model**:

  ```console
  rotate X 45
  ```

- **Translating the Model**:

  ```console
  translate Y 10
  ```

- **Exporting the Model**:

  ```console
  save SimpleCube.json
  ```

## Sample Inputs and Outputs

### Test Case 1: Model with Holes

#### Description
Validate the system’s ability to handle BREP models containing holes within faces, ensuring correct loop associations and genus calculations.

#### Input
`HoleTest.json` located in the `json_files/` directory.

#### Commands

```console
import HoleTest.json
render HoleTest.json
```

#### Expected Output

- Successful loading of the model.
- Accurate identification of outer and inner loops.
- Correct genus calculation reflecting the number of holes.
- A rendered image saved in the `render/test_outputs/` directory.

### Test Case 2: Simple Cube

#### Description
Verify the system’s ability to handle a basic BREP model without any holes, ensuring fundamental functionalities work as expected.

#### Input
`SimpleTest.json` located in the `json_files/` directory.

#### Commands

```console
import SimpleTest.json
render SimpleTest.json
```

### Test Case 3: Non-Closed Loop

#### Description
Ensure that the system correctly identifies and rejects loops that are not closed.

#### Input
`OpenLoops.json` located in the `json_files/` directory.

#### Commands

```console
import OpenLoops.json
```

#### Expected Output

- Error message indicating that the loop is not closed.
- The model is rejected to maintain structural integrity.

## Known Limitations

While the 3D BREP Software System offers robust functionalities for 3D modeling and validation, certain limitations exist:

1. **Input Format Rigidity**:
   - The system exclusively accepts inputs in JSON format.
