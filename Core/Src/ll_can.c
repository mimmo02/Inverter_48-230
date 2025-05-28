/* ll_CAN.c

 *
 * 		Interface � la librairie CAN
 */
#include "ll_can.h"
#ifdef STM32H743xx
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_fdcan.h>
#else
#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_can.h>
#endif
#include <string.h>
//#include "pelab_can_functions.h"
//#include "ll_handles.h"
//#include "shared/shared_converter_cst.h"
#include <stdbool.h>


