#include "cli_controller.h"
#include <iostream>
#include <sstream>
#include <string>

void CLIController::run() {
    std::string command;
    while (true) {
        std::cout << "> ";
        std::getline(std::cin, command);
        if (command == "exit") {
            break;
        }
        parseCommand(command);
    }
}

void CLIController::parseCommand(const std::string& cmd) {
    if (cmd == "help") {
        displayHelp();
    } else if (cmd.find("load brep") == 0) {
        std::string jsonInput = cmd.substr(9);
        if (jsonInput.empty()) {
            std::cout << "Error: No JSON data provided." << std::endl;
            return;
        }
        try {
            json j = json::parse(jsonInput);
            BREP brep = BREP::fromJSON(j);
            operations.loadBREP(brep);
            std::cout << "BREP loaded successfully." << std::endl;
        } catch (const json::parse_error& e) {
            std::cout << "JSON Parsing Error: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cout << "Error: " << e.what() << std::endl;
        }
    } else if (cmd.find("rotate") == 0) {
        std::istringstream iss(cmd);
        std::string command, axis;
        float theta;
        iss >> command >> axis >> theta;
        try {
            operations.rotate(axis, theta);
            std::cout << "BREP rotated successfully." << std::endl;
        } catch (const std::exception& e) {
            std::cout << "Error: " << e.what() << std::endl;
        }
    } else if (cmd.find("translate") == 0) {
        std::istringstream iss(cmd);
        std::string command, axis;
        float units;
        iss >> command >> axis >> units;
        try {
            operations.translate(axis, units);
            std::cout << "BREP translated successfully." << std::endl;
        } catch (const std::exception& e) {
            std::cout << "Error: " << e.what() << std::endl;
        }
    } else if (cmd == "display brep") {
        operations.displayCurrentBREP();
    } else if (cmd.find("project") == 0) {
        std::istringstream iss(cmd);
        std::string command, view;
        iss >> command >> view;
        try {
            operations.orthogonalProjection(view);
        } catch (const std::exception& e) {
            std::cout << "Error: " << e.what() << std::endl;
        }
    } else if (cmd == "undo") {
        operations.undo();
    } else if (cmd == "redo") {
        operations.redo();
    } else if (cmd.find("load json") == 0) {
        std::istringstream iss(cmd);
        std::string command, filename;
        iss >> command >> filename;
        if (filename.empty()) {
            std::cout << "Error: No filename provided." << std::endl;
            return;
        }
        try {
            operations.loadBREPFromFile(filename);
        } catch (const std::exception& e) {
            std::cout << "Error loading BREP: " << e.what() << std::endl;
        }
    } else if (cmd.find("save") == 0) {
        std::istringstream iss(cmd);
        std::string command, filename;
        iss >> command >> filename;
        try {
            if (filename.empty()) {
                operations.save();  // Use default filename
            } else {
                operations.save(filename);
            }
        } catch (const std::exception& e) {
            std::cout << "Error: " << e.what() << std::endl;
        }
    } else if (cmd == "reset") {
        operations.reset();
        std::cout << "BREP reset to initial state." << std::endl;
    } else if (cmd.find("import") == 0) {
        // Handle 'import' command
        size_t pos = cmd.find("import") + 6;
        // Skip any whitespace
        while (pos < cmd.size() && std::isspace(cmd[pos])) {
            pos++;
        }
        // Extract filename
        std::string filename = cmd.substr(pos);
        if (filename.empty()) {
            std::cout << "Error: No filename provided for import." << std::endl;
            return;
        }
        try {
            operations.importBREP(filename);
            std::cout << "BREP imported successfully from " << filename << "." << std::endl;
        } catch (const std::exception& e) {
            std::cout << "Error importing BREP: " << e.what() << std::endl;
        }
    } else if (cmd.find("render") == 0) {
        std::istringstream iss(cmd);
        std::string command, filename;
        iss >> command >> filename;
        try {
            if (filename.empty()) {
                std::cout << "Please provide a filename to save the rendering data (e.g., 'render brep_render')." << std::endl;
            } else {
                operations.render(filename);
            }
        } catch (const std::exception& e) {
            std::cout << "Error rendering BREP: " << e.what() << std::endl;
        }
    }else if (cmd.find("cross section") == 0) {
        std::istringstream iss(cmd);
        std::string command, subcommand;
        iss >> command >> subcommand;

        std::string a_str, b_str, c_str, d_str, sign;
        iss >> a_str >> b_str >> c_str >> d_str >> sign;

        try {
            float a = std::stof(a_str);
            float b = std::stof(b_str);
            float c = std::stof(c_str);
            float d = std::stof(d_str);
            Plane planeEqn(a, b, c, d);
            operations.crossSection(planeEqn, sign);
            // No need to print success message here; it's handled in the method
        } catch (const std::exception& e) {
            std::cout << "Error parsing cross section command: " << e.what() << std::endl;
            std::cout << "Usage: cross section [a] [b] [c] [d] [positive|negative]" << std::endl;
        }}}

void CLIController::displayHelp() {
    std::cout << "Available commands:" << std::endl;
    std::cout << "  help          - Display this help message" << std::endl;
    std::cout << "  import        - Import BREP from a JSON file" << std::endl;
    std::cout << "  load brep     - Load BREP from a JSON string" << std::endl;
    std::cout << "  save          - Save current BREP to a JSON file" << std::endl;
    std::cout << "  display brep  - Display the current BREP data" << std::endl;
    std::cout << "  rotate        - Rotate the BREP around an axis" << std::endl;
    std::cout << "  translate     - Translate the BREP along an axis" << std::endl;
    std::cout << "  project       - Apply an orthogonal projection" << std::endl;
    std::cout << "  cross section - Perform a cross-section operation" << std::endl;
    std::cout << "  render        - Render the current BREP to an image" << std::endl;
    std::cout << "  undo          - Undo the last operation" << std::endl;
    std::cout << "  redo          - Redo the last undone operation" << std::endl;
    std::cout << "  reset         - Reset to the initial loaded BREP" << std::endl;
    std::cout << "  exit          - Exit the application" << std::endl;
}
