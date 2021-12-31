
# ADS1255 / ADS1256 Library

### Very Low Noise, 24-Bit Analog-to-Digital Converter

This library is made for the STM32 HAL (Hardware Abstraction Library) platform. The example code is for STM32CubeMX and Keil uVision 5 IDE.

> **Datasheet: [ti.com](https://www.ti.com/lit/ds/sbas288k/sbas288k.pdf?)**

---

[![forthebadge](https://forthebadge.com/images/badges/made-with-c.svg)](https://forthebadge.com)

### Usage

```c
/* USER CODE BEGIN Includes */
#include <ads1255.h>
```
```c
/* USER CODE BEGIN PV */
ADS125X_t adc1;
/* USER CODE END PV */

int main(void)
{
    /* USER CODE BEGIN 2 */
    adc1.csPort = SPI2_CS_GPIO_Port;
    adc1.csPin  = SPI2_CS_Pin;
    adc1.drdyPort = SPI2_DRDY_GPIO_Port;
    adc1.drdyPin  = SPI2_DRDY_Pin;
    adc1.vref = 2.5f;
    adc1.oscFreq = ADS125X_OSC_FREQ;
    
    printf("\n");
    printf("ADC config...\n");
    // 10 SPS / PGA=1 / buffer off
    ADS125X_Init(&adc1, &hspi2, ADS125X_DRATE_10SPS, ADS125X_PGA1, 0);
    printf("...done\n");
    /* USER CODE END 2 */


    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        float volt[2] = {0.0f, 0.0f};
        ADS125X_ChannelDiff_Set(&adc1, ADS125X_MUXP_AIN0, ADS125X_MUXN_AIN1);
        volt[0] = ADS125X_ADC_ReadVolt(&adc1);
        printf("%.15f\n", volt[0]);
        
        ADS125X_ChannelDiff_Set(&adc1, ADS125X_MUXP_AIN2, ADS125X_MUXN_AIN3);
        ADS125X_CMD_Send(&adc1, ADS125X_CMD_SYNC);
        volt[1] = ADS125X_ADC_ReadVolt(&adc1);
        printf("%.15f\n", volt[1]);
        
    }
    /* USER CODE END 3 */
}

```

---

### See Also

- [adienakhmad/ADS1256](https://github.com/adienakhmad/ADS1256)
- [Flydroid/ADS12xx-Library](https://github.com/Flydroid/ADS12xx-Library)
- [mbilsky/TeensyADS1256](https://github.com/mbilsky/TeensyADS1256)

---

### License 

Apache 2.0 

(c) 2022 eta Systems GmbH

