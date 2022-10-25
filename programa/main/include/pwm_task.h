/**
 * @file pwm_task.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

void pwm_task();

typedef enum Item {
    eNone,
    ePwmItem_1,
    ePwmItem_2,
} eItem;

void choose_item(eItem item);