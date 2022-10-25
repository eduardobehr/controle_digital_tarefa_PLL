/**
 * @file interpreter.h
 * @author Eduardo Eller Behr (eellerbehr@gmail.com)
 * @brief 
 * @date 2022-09-14
 * 
 * @copyright (c) 2022
 * 
 */
#pragma once
#include <string.h>
#include <stdint.h>

#define cCommandLineArgsNum 50
#define cResponseLineSize 1024
#define cCommandLineBufferSize 1024

enum Status_t {
    eOk,
    eInvalidArg,
    eInvalidValue,
    eNullPointerException,
    eMemoryError,
    eNeedsDeallocation,     /// Houve uma alocação dinâmica que precisa ser liberada externamente
};

enum CommandsEnum {
    eUnknown,
    eSetRegisterValue,
    eGetRegisterValue,
    eHelp,
    ePrint,
    eRestart,
    eExit,
};

typedef struct Command Command_t;

typedef struct CommandLine {
    Command_t* cmd;                    /// CommandsEnum
    char* args[cCommandLineArgsNum];
} CommandLine_t;

typedef struct Command {
    // int command_id;
    const char* const command_name;                        
    int (* const func) (const CommandLine_t* command_line, char** pResponse); 
    const char* const description;
} Command_t;

/**
 * @brief Receives a command string and returns a command code
 * 
 * @param word 
 * @return int 
 */
Command_t* interpret_command(const char* word);

/**
 * @brief Deallocates the memory of all arguments of a command line
 * 
 * @param command_line 
 */
void free_command_line_arguments(CommandLine_t* command_line);


/**
 * @brief Processa um buffer de texto e retorna um commando interpretado
 * 
 * @param buffer 
 * @return CommandLine_t 
 */
CommandLine_t parse_command_line(char* buffer);

/**
 * @brief executa um comando CommandsEnum e retorna o estado (Status_t)
 * 
 * @param cmd 
 * @return int (Status_t)
 */
int execute_command(const CommandLine_t* pCmd, char** pResponse);

/**
 * @brief Makes command available to be easily recalled
 * 
 * @param pCmd 
 */
// void add_to_history(const CommandLine_t* pCmd);