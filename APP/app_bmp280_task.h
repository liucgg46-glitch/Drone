#ifndef __APP_BMP280_TASK_H
#define __APP_BMP280_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t App_BMP280_Init(void);
void App_BMP280_RegisterTasks(void);

void task_bmp280_read(void);
void task_bmp280_print(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BMP280_TASK_H */
