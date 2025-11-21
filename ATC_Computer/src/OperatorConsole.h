#ifndef OPERATORCONSOLE_H_
#define OPERATORCONSOLE_H_

#pragma once
#include "CommunicationsSystem.h"
#include <iostream>
#include <sys/dispatch.h>
#include <thread>
#include "Msg_structs.h"

class OperatorConsole {
public:
	OperatorConsole(CommunicationsSystem& comms);
    ~OperatorConsole();
    void start();

private:
    void HandleConsoleInputs();
    void logCommand(const std::string& command);
    std::thread Operator_Console;
    bool exit = false;
    CommunicationsSystem& commsRef;
};



#endif /* OPERATORCONSOLE_H_ */
