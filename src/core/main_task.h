#ifndef MAINTASK_H_
#define MAINTASK_H_

#include "FreeRTOS.h"
#include "task.h"

void initMainTask();
void MainTask(void *pvParameters);

#endif // MAINTASK_H_
