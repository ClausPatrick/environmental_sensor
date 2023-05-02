Pico reading sensor data from:
AM2320 (Temperature and humidity);
SGP40 (Volatile Organic Compounds);
PMSA003i (Particle detection);
Data is displayed on LCD (16x4) and VOC is indicated by RGB LED. U2U protocol is used for Central to read out data.
Display driver causes issue with the sleep routine so it is seperated out for core_1 to take care of.
