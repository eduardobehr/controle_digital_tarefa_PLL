/**
 * @file interpreter.c
 * @author Eduardo Eller Behr (eellerbehr@gmail.com)
 * @brief 
 * @date 2022-09-14
 * 
 * @copyright (c) 2022
 * 
 */

#include <interpreter.h>
// #include <pwm_task.h>

#include <soc/mcpwm_reg.h>
#include <soc/dport_reg.h>
#include <soc/gpio_reg.h>

#include <esp_system.h>
#include <esp_log.h>

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



CommandLine_t parse_command_line(char* buffer){


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
        token = strtok(NULL, cDelimiter);

        if(token == NULL){
            break;
        }

        // processar argumento
        if(counter <= cCommandLineArgsNum){
            size_t arg_size = strlen(token);
            char* arg_heap_alloc = malloc(arg_size);
            memcpy(arg_heap_alloc, token, arg_size);
            arg_heap_alloc[arg_size] = '\0';

            command_line.args[counter] = arg_heap_alloc;

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

static int help(const CommandLine_t* command_line, char** pResponse);

static int restart(const CommandLine_t* command_line, char** pResponse){
    printf("  Reiniciando (comando de usuário)\n");
    esp_restart();
    return eOk;
}

static int free_heap(const CommandLine_t* command_line, char** pResponse){
    uint32_t free_heap = esp_get_free_heap_size();
    const char* format = "  %u bytes disponíveis na heap\n";
    sprintf(*pResponse, format, free_heap);
    printf(*pResponse);
    return eOk;
}

static int uptime(const CommandLine_t* command_line, char** pResponse){
    uint32_t sys_milliseconds = esp_log_timestamp();

    uint32_t sys_seconds = (sys_milliseconds % 60000)/1000;
    uint32_t sys_minutes = (sys_milliseconds % 3600000)/60000;
    uint32_t sys_hours =  sys_milliseconds/3600000;

    const char* format = "  Sistema em execução há %uh %umin %us\n";
    sprintf(*pResponse, format, sys_hours, sys_minutes, sys_seconds);
    return eOk;

}

static int unknown(const CommandLine_t* command_line, char** pResponse){
    *pResponse = "  Comando desconhecido. Digite 'help' para saber mais\n";
    return eUnknown;
}

static int set_register(const CommandLine_t* command_line, char** pResponse){
    char* pReg = command_line->args[0];
    char* pVal = command_line->args[1];

    if(!pReg || !pVal){
        *pResponse = "  Argumentos insuficientes: setreg <REGISTRADOR> <VALOR>\n";
        return eInvalidArg;
    }
    

    printf("SET: %s = %s\n", pReg, pVal);

    
    uint32_t reg = strtoul(pReg, NULL, 0);
    uint32_t val = strtoul(pVal, NULL, 0);

    // TODO: limitar intervalo ao dos periféricos
    if(reg == 0){
        *pResponse = "Falha na conversão de argumento\n";
        return eInvalidArg;
    }

    *((volatile uint32_t*) reg) = val;

    const char* format = "Escrevendo em registrador:\n  *((volatile uint32_t*) 0x%x) = %u\n";
    snprintf(*pResponse, cResponseLineSize, format, reg, val);

    return eOk;
}
/**
 * @brief Configura bits específicos em um registrador.
 * Argumentos: <ENDEREÇO> <DESLOCAMENTO> <NBITS> <VALOR>
 * 
 * @param command_line 
 * @param pResponse 
 * @return int 
 */
static int set_bits(const CommandLine_t* command_line, char** pResponse){
    char* pReg = command_line->args[0];
    char* pOffset = command_line->args[1];
    char* pLength = command_line->args[2];
    char* pVal = command_line->args[3];

    if(!pReg || !pOffset || !pLength || !pVal){
        *pResponse = "  Argumentos insuficientes: setbits <ENDEREÇO> <DESLOCAMENTO> <NBITS> <VALOR>\n";
        return eInvalidArg;
    }
    
    // printf("SETBITS: %s = %s\n", pReg, pVal);

    uint32_t reg = strtoul(pReg, NULL, 0);
    uint32_t offset = strtoul(pOffset, NULL, 0);
    uint32_t length = strtoul(pLength, NULL, 0);
    uint32_t val = strtoul(pVal, NULL, 0);

    // Verificar se valores de offset e length são válidos
    if(offset+length > 32){
        *pResponse = "Argumentos inválidos: Soma de <DESLOCAMENTO> e <NBTIS> não pode ser maior que 32\n";
        return eInvalidArg;
    }

    // TODO: limitar intervalo ao dos periféricos
    if(reg == 0){
        *pResponse = "Falha na conversão de argumento\n";
        return eInvalidArg;
    }

    // Montar máscara binária. Ex: se offset=2, length=4 => bitmask=0b00111100
    uint32_t bitmask = 0;
    for (size_t i = offset; i < (length+offset); i++){
        bitmask |= 1 << i;
    }

    //                   __________Zerar valor antigo______________   ____Calcular valor novo____
    uint32_t new_value = ((*((volatile uint32_t*) reg)) & ~bitmask) | ((val << offset) & bitmask);

    // Executar escrita
    *((volatile uint32_t*) reg) = new_value;

    // Montar resposta  
    const char* format = "Escrevendo em registrador:\n  *((volatile uint32_t*) 0x%x) = %u << %u\n";
    snprintf(*pResponse, cResponseLineSize, format, reg, val, offset);

    return eOk;
}

static int get_register(const CommandLine_t* command_line, char** pResponse){
    char* pReg = command_line->args[0];
    
    if(!pReg){
        const char* format = "Argumento não especificado: %s <REGISTRADOR>\n";
        snprintf(*pResponse, cResponseLineSize, format, command_line->cmd->command_name);

        return eInvalidArg;
    }

    uint32_t reg = strtoul(pReg, NULL, 0);

    // TODO: limitar intervalo ao dos periféricos
    if(reg == 0){
        *pResponse = "Falha na conversão de argumento\n";
        return eInvalidArg;
    }

    uint32_t val = *((volatile uint32_t*) reg);


    const char* format = "Lendo de registrador:\n  *((volatile uint32_t*) 0x%lx) == %lu\n";
    snprintf(*pResponse, cResponseLineSize, format, reg, val);
    return eOk;
}

static int choose(const CommandLine_t* command_line, char** pResponse){
    const char* arg_tarefa = command_line->args[0];
    const char* arg_item = command_line->args[1];

    if(arg_tarefa == NULL || arg_item == NULL){
        sprintf(*pResponse, "São necessários dois argumentos: <tarefa> e <item>");
        return eInvalidArg;
    }

    if(strcmp("pwm", arg_tarefa) == 0){
        if(strcmp("0", arg_item) == 0 || strcmp("off", arg_item) == 0){
            // para execução
            choose_item(eNone);
            sprintf(*pResponse, "  Desativando pwm");
            return eOk;
        }
        else if(strcmp("1", arg_item) == 0){
            choose_item(ePwmItem_1);
        }
        else if(strcmp("2", arg_item) == 0){
            choose_item(ePwmItem_2);
        }
        else{
            sprintf(*pResponse, "  Argumento '%s' inválido!", arg_item);
            return eInvalidArg;
        }

        sprintf(*pResponse, "  Executando item %s da tarefa do pwm.", arg_item);
        return eOk;
    }
    else if(strcmp("adc", arg_tarefa) == 0){
        // TODO:
        sprintf(*pResponse, "  Não implementado!");
        return eOk;
    }
    else{
        sprintf(*pResponse, "  Argumento '%s' inválido!", arg_tarefa);
        return eInvalidArg;
    }
    
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

static Command_t cmd_set_bits = {
    .command_name = "setbits",
    .func = set_bits,
    .description = "Modifica valores de bits no registrador. \n  Argumentos: <ENDEREÇO> <DESLOCAMENTO> <NBITS> <VALOR>",
};

static Command_t cmd_get_reg = {
    .command_name = "getreg",
    .func = get_register,
    .description = "Lê e retorna o valor do registrador. \n  Argumentos: <ENDEREÇO>",
};

static Command_t cmd_restart = {
    .command_name = "restart",
    .func = restart,
    .description = "Reinicia o sistema.",
};

static Command_t cmd_free_heap = {
    .command_name = "free", 
    .func = free_heap,
    .description = "Retorna a quantidade de RAM (heap) disponível.",
};

static Command_t cmd_history = {
    .command_name = "history",
    .func = NULL,
    .description = "TODO: Retorna os comandos anteriores.",
};

static Command_t cmd_uptime = {
    .command_name = "uptime",
    .func = uptime,
    .description = "Retorna o tempo ativo do sistema.",
};

static Command_t cmd_choose_problem = {
    .command_name = "choose",
    .func = choose,
    .description = "Escolhe o programa para executar, de acordo com a questão da tarefa escolhida.\n  Args: [pwm] [0..2]",
};


#define cCommandsCount 20

static Command_t* all_commands[cCommandsCount] = {
    /// @note: o comando 'unknown' não precisa estar listado aqui
    &cmd_help,
    &cmd_set_reg,
    &cmd_restart,
    &cmd_get_reg,
    &cmd_free_heap,
    &cmd_uptime,
    &cmd_set_bits,
    &cmd_choose_problem,
    NULL // manter ao fim para limitar laço em interpret_command
};

static int help(const CommandLine_t* command_line, char** pResponse){
    char buff[cResponseLineSize];
    memset(buff, ' ', cResponseLineSize);

    size_t commands_count = 0;
    // calcular número verdadeiro de comandos e preparar buffer de formatação
    for (size_t i = 0; i < cCommandsCount; i++){
        if(all_commands[i] == NULL){
            commands_count = i;
            break;
        }
        
    }

    volatile size_t current_index = 0;
    volatile size_t total_line_size = 0;
    volatile size_t previous_total_line_size = 0;
    for (size_t i = 0; i < commands_count; i++){
        if(all_commands[i] == NULL){
            break;
        }
        const char* line_format = "\n=>\033[32;1;4m%s\033[0m: %s\n";
        size_t format_size = strlen(line_format);
        size_t name_size = strlen(all_commands[i]->command_name);
        size_t description_size = strlen(all_commands[i]->description);
        total_line_size = name_size + description_size + format_size;

        char* line_buffer = malloc(total_line_size);
        memset(line_buffer, ' ', total_line_size);


        if(line_buffer != NULL){
            
            snprintf(line_buffer, total_line_size, line_format, 
                all_commands[i]->command_name, all_commands[i]->description);

            memcpy(&buff[current_index], line_buffer, total_line_size);
            
            previous_total_line_size = total_line_size;
            free(line_buffer);
        }

        current_index += previous_total_line_size-3; // FIXME: unknown magic number

    }

    // Gambiarra para remediar bug de só printar a primeira linha
    for (size_t i = 0; i < cResponseLineSize; i++){
        if(buff[i] == '\0'){
            buff[i] = ' ';
        }
    }

    buff[current_index] = '\0';

    memcpy(*pResponse, buff, cResponseLineSize);

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

typedef struct Node Node_t;

struct Node {
    CommandLine_t cmd;
    Node_t* __next__;
};

// TODO: parar de chamar free_command_line_arguments (fazer um comando 'clean' pra esvaziar o histórico)
// void add_to_history(const CommandLine_t* pCmd){
//     static Node_t* next = NULL;
//     // TODO: criar linked list
    
//     Node_t node = {
//         .cmd = *pCmd,
//         .__next__ = NULL
//     };
// }

int execute_command(const CommandLine_t* pCmd, char** pResponse){
    return pCmd->cmd->func(pCmd, pResponse); 
}