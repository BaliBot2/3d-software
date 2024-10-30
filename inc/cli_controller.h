#ifndef CLI_CONTROLLER_H
#define CLI_CONTROLLER_H

#include "brep_operations.h"
#include <string>

class CLIController {
private:
    BREPOperations operations;

public:
    void run();

    void parseCommand(const std::string& cmd);

    void displayHelp();
};

#endif // CLI_CONTROLLER_H
