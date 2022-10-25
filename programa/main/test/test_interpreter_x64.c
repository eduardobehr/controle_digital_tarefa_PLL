/**
 * @file test_interpreter_x64.c
 * @author Eduardo Eller Behr (eellerbehr@gmail.com)
 * @brief 
 * @date 2022-09-14
 * 
 * @copyright (c) 2022
 * 
 */
#include "interpreter_prvt.h"
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

static int strings_are_equal(const char* s1, const char* s2){
    if(s1 && s2) {
        return !strcmp(s1, s2);
    } else {
        return -1;
    }
}

#define cResponseLineSize 512
#define cCommandLineBufferSize 1024

CommandLine_t parse_command_line(char* buffer){
    
    // printf("Input buffer: '%s'\n", buffer_copy);

    // dividir a string em palavras (separadas por espaço em branco)
    const char* cDelimiter = " ";

    // Remover caractere de nova linha do final
    size_t length = strlen(buffer);
    if(buffer[length-1] == '\n'){
        buffer[length-1] = '\0';
    }

    char* token = strtok(buffer, cDelimiter);
    
    // criar objeto de retorno (CommandLine_t)
    CommandLine_t command_line = {};

    // primeira palavra é o comando
    command_line.cmd = interpret_command(token);

    int counter = 0;
    while(1) {
        // próximo argumento
        // printf(" %i: %s\n", counter-1, token);
        token = strtok(NULL, cDelimiter);

        if(token == NULL){
            break;
        }

        // processar argumento
        if(counter <= cCommandLineArgsNum){
            size_t arg_size = strlen(token);
            char* arg_heap_alloc = malloc(arg_size);
            memcpy(arg_heap_alloc, token, arg_size);

            command_line.args[counter] = arg_heap_alloc;
            // printf("command_line.args[%i] = %s (%p)\n", counter, token, token);
        } else {
            printf("Argumentos excessivos\n");
            break;
        }
        counter++;
    }

    return command_line;
}

void free_command_line_arguments(CommandLine_t* command_line){
    for (size_t i = 0; i < cCommandLineArgsNum; i++){
        char* p_argument = command_line->args[i];
        if(p_argument != NULL){
            free(p_argument);
        }
    }
}

static int help(CommandLine_t* command_line, char** pResponse);

static int _exit_(CommandLine_t* command_line, char** pResponse){
    printf("  Finalizando\n");
    exit(0);
    return eOk;
}

static int unknown(CommandLine_t* command_line, char** pResponse){
    *pResponse = "Comando desconhecido. Digite 'help' para saber mais";
    return eUnknown;
}

static int set_register(CommandLine_t* command_line, char** pResponse){
    char* pReg = command_line->args[0];
    char* pVal = command_line->args[1];

    if(!pReg || !pVal){
        *pResponse = "Argumentos insuficientes: setreg <REGISTRADOR> <VALOR>";
        return eInvalidArg;
    }
    

    printf("SET: %s = %s\n", pReg, pVal);

    uint32_t reg = strtoul(pReg, NULL, 0);
    uint32_t val = strtoul(pVal, NULL, 0);
    if(reg == 0){
        *pResponse = "Falha na conversão de argumento";
        return eInvalidArg;
    }

    const char* format = "Setting register:  *((volatile uint32_t*) 0x%x) = %u";
    snprintf(*pResponse, cResponseLineSize, format, reg, val);

    return eOk;
}

static int get_register(CommandLine_t* command_line, char** pResponse){
    char* pReg = command_line->args[0];
    
    if(!pReg){
        const char* format = "Argumento não especificado: %s <REGISTRADOR>";
        snprintf(*pResponse, cResponseLineSize, format, command_line->cmd->command_name);

        return eInvalidArg;
    }

    uint32_t reg = strtoul(pReg, NULL, 0);
    printf("GET: *(0x%x)\n", reg);

    *pResponse = malloc(64);
    if(*pResponse != NULL){
        
        uint32_t val = 123456;// TODO:

        const char* format = "TODO: NÃO IMPLEMENTADO: Getting register: *((volatile uint32_t*) 0x%x) == %u";
        snprintf(*pResponse, cResponseLineSize, format, reg, val);
        return eOk;
    }
    return eMemoryError;
}

static Command_t cmd_unknown = {
    .command_name = "unknown",
    .func = unknown
};

static Command_t cmd_help = {
    .command_name = "help",
    .func = help,
    .description = "Informações dos demais comandos.",
};

static Command_t cmd_set_reg = {
    .command_name = "setreg",
    .func = set_register,
    .description = "Escreve um valor no registrador. \n  Argumentos: <ENDEREÇO> <VALOR>",
};

static Command_t cmd_get_reg = {
    .command_name = "getreg",
    .func = get_register,
    .description = "Lê e retorna o valor do registrador. \n  Argumentos: <ENDEREÇO>",
};

static Command_t cmd_exit = {
    .command_name = "exit",
    .func = _exit_,
    .description = "Finaliza sessão do interpretador.",
};


#define cCommandsCount 20

static Command_t* all_commands[cCommandsCount] = {
    /// @note: o comando 'unknown' não precisa estar listado aqui
    &cmd_help,
    &cmd_set_reg,
    &cmd_exit,
    &cmd_get_reg,
    NULL // manter ao fim para limitar laço em interpret_command
};

static int help(CommandLine_t* command_line, char** pResponse){
    char buff[cResponseLineSize];
    memset(buff, ' ', cResponseLineSize);
    // const char* line_format = " - %s: %s\n\n"; // command, description
    // char line_buffer[cCommandLineBufferSize];

    // const char* buffer_formatter_repetition = "%s\n";
    // size_t formatter_len = strlen(buffer_formatter_repetition);

    size_t commands_count = 0;
    // calcular número verdadeiro de comandos e preparar buffer de formatação
    for (size_t i = 0; i < cCommandsCount; i++){
        if(all_commands[i] == NULL){
            commands_count = i;
            break;
        }

        // char* current_pointer = &buff[i*(formatter_len)];

        // memcpy(current_pointer, buffer_formatter_repetition, formatter_len);
        
    }

    size_t current_index = 0;
    size_t total_line_size = 0;
    size_t previous_total_line_size = 0;
    for (size_t i = 0; i < commands_count; i++){
        if(all_commands[i] == NULL){
            break;
        }
        

        const char* line_format = "%s: %s\n";
        size_t format_size = strlen(line_format);
        size_t name_size = strlen(all_commands[i]->command_name);
        size_t description_size = strlen(all_commands[i]->description);
        total_line_size = name_size + description_size + 2;
        // FIXME:

        char* line_buffer = malloc(total_line_size+5);
        memset(line_buffer, 0, total_line_size);


        if(line_buffer != NULL){
            
            snprintf(line_buffer, total_line_size+format_size, line_format, 
                all_commands[i]->command_name, all_commands[i]->description);

            memcpy(&buff[current_index], line_buffer, total_line_size);
            
            previous_total_line_size = total_line_size+format_size;

            free(line_buffer);
        }
        


        // snprintf(line_buffer, cCommandLineBufferSize, line_format, 
        //     all_commands[i]->command_name, all_commands[i]->description);
        current_index += previous_total_line_size;
        memset(&buff[current_index-1], '\n', 1);
    }

    buff[current_index] = '\0';
    // printf("%s", buff);
    // *pResponse = buff;
    memcpy(*pResponse, buff, cResponseLineSize);
    // *pResponse = "HELP...";
    // fwrite("hahaha", cResponseLineSize, cResponseLineSize, *pResponse);
    return eOk;
}

Command_t* interpret_command(const char* word){

    for (size_t i = 0; i < cCommandsCount; i++){
        if(all_commands[i] == NULL){
            break;
        }
        if(strings_are_equal(word, all_commands[i]->command_name)){
            return all_commands[i];
        }
    }

    return &cmd_unknown;

}



int execute_command(CommandLine_t* pCmd, char** pResponse){
    return pCmd->cmd->func(pCmd, pResponse);
}

int main(){

    
    char command_line_buffer[cCommandLineBufferSize] = {};

    char* pResponse;
    char default_return_buffer[cResponseLineSize];

    while (1){
        CommandLine_t cmdline = {};
        pResponse = default_return_buffer;

        memset(&cmdline, 0, sizeof(cmdline));
        printf("→ ");

        fgets(command_line_buffer, cCommandLineBufferSize, stdin);

        cmdline = parse_command_line(command_line_buffer);
        // printf("Parsed command name: %s\n", cmdline.cmd->command_name);
        // printf("arg 0: %s\n", cmdline.args[0]);
        // printf("arg 1: %s\n...\n", cmdline.args[1]);

        int status = execute_command(&cmdline, &pResponse);

        printf("   ← %s\n", pResponse);
        // Após executar comando, liberar buffers se necessário
        free_command_line_arguments(&cmdline);

        if(status == eNeedsDeallocation){
            free(pResponse);
        }



    }

    return 0;
}