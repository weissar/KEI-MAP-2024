# MEMS XNucleo Shieldy

* Pro Nucleo desky existuje řada "originálních" shieldů s různými MEMS senzoty - typicky víceosý akcelerometr, gyroskop, magnetometr. teploměr/tlakoměr, příp. i mikrofon a další
* Úkolem práce je:
  * **prozkoumat dokumentaci, nastavit pracovní režim, periodicky vyčítat hodnoty a zobrazovat pomocí terminálu**
  * Pokud zbydou shieldy s displejem, bylo by možné použít pro zobrazení i ten
  * Při komunikaci s terminálem by bylo vhodné i některé parametry nastavvoat = nap.ř. citlivost akcelerometru.
* Některé jsou k dispozici - nutno domluvit.
* Přehled dle webu STM - aktivní odkazy + dokumentace:
  * [X-NUCLEO-IKS01A2](https://www.st.com/en/ecosystems/x-nucleo-iks01a2.html)
    * LSM6DSL MEMS 3D accelerometer (±2/±4/±8/±16 g) and 3D gyroscope (±125/±245/±500/±1000/±2000 dps)
    * LSM303AGR MEMS 3D accelerometer (±2/±4/±8/±16 g) and MEMS3D magnetometer (±50 gauss)
    * LPS22HB MEMS pressure sensor, 260-1260 hPa absolute digital output barometer
    * HTS221: capacitive digital relative humidity and temperature
  * [X-NUCLEO-IKS01A1](https://www.st.com/en/ecosystems/x-nucleo-iks01a1.html)
    * LSM6DS0: MEMS 3D accelerometer (±2/±4/±8 g) + 3D gyroscope (±245/±500/±2000 dps)
    * LIS3MDL: MEMS 3D magnetometer (±4/ ±8/ ±12/ 16 gauss)
    * LPS25HB*: MEMS pressure sensor, 260-1260 hPa absolute digital output barometer
    * HTS221: capacitive digital relative humidity and temperature
  * [X-NUCLEO-IKS01A3](https://www.st.com/en/ecosystems/x-nucleo-iks01a3.html)
    * LSM6DSO: MEMS 3D accelerometer (±2/±4/±8/±16 g) + 3D gyroscope (±125/±250/±500/±1000/±2000 dps)
    * LIS2MDL: MEMS 3D magnetometer (±50 gauss)
    * LIS2DW12: MEMS 3D accelerometer (±2/±4/±8/±16 g)
    * LPS22HH: MEMS pressure sensor, 260-1260 hPa absolute digital output barometer
    * HTS221: capacitive digital relative humidity and temperature
    * STTS751: Temperature sensor (–40 °C to +125 °C)
  * [X-NUCLEO-IKS4A1](https://www.st.com/en/ecosystems/x-nucleo-iks4a1.html)
    * LSM6DSO16IS: MEMS 3D accelerometer (±2/±4/±8/±16 g) + 3D gyroscope (±125/±250/±500/±1000/±2000 dps) with ISPU (Intelligent Processing Unit)
    * LIS2MDL: MEMS 3D magnetometer (±50 gauss)
    * LIS2DUXS12: Ultra low-power MEMS 3D accelerometer (±2/±4/±8/±16 g) with Qvar, AI, & anti-aliasing
    * LPS22DF: Low-power and high-precision MEMS pressure sensor, 260-1260 hPa absolute digital output barometer
    * SHT40AD1B: High-accuracy, ultra-low-power relative humidity and temperature sensor (by Sensirion)
    * STTS22H: Low-voltage, ultralow-power, 0.5 °C accuracy temperature sensor (–40 °C to +125 °C)
    * LSM6DSV16X: MEMS 3D accelerometer (±2/±4/±8/±16 g) + 3D gyroscope (±125/±250/±500/±1000/±2000/±4000 dps) with embedded sensor fusion, AI, Qvar
    * LSM6DSV16X: MEMS 3D accelerometer (±2/±4/±8/±16 g) + 3D gyroscope (±125/±250/±500/±1000/±2000/±4000 dps) with embedded sensor fusion, AI, Qvar
  * [X-NUCLEO-IKS02A1](https://www.st.com/en/ecosystems/x-nucleo-iks02a1.html)
    * ISM330DHCX MEMS 3D accelerometer (±2/±4/±8/±16 g) plus 3D gyroscope (±125/±250/±500/±1000/±2000 dps)
    * IIS2MDC MEMS 3D magnetometer (±50 gauss)
    * IIS2DLPC MEMS 3D accelerometer low power (±2/±4/±8/±16 g)
    * IMP34DT05 MEMS digital omnidirectional microphone (-26 dBFS, ±3 dB sensitivity)

## Knihovní soubory:
* Používá se v podstatě jen knihovna pro I2C komunikaci
* Obvody senzorů navíc zpravidla používají některé GPIO pro informaci o stavu - typicky signál "data-ready"
