/* Readable sensor/mode diagnostics on USART1 TX (PA9), 115200 baud, 8N1. */
#ifndef HAPTIC_DIAGNOSTICS_H
#define HAPTIC_DIAGNOSTICS_H

/* Initialize once after the 48 MHz system clock and generated GPIO setup. */
void HapticDiagnostics_Init(void);

/* Call from the main loop; emits at most one status record per second. */
void HapticDiagnostics_Poll(void);

#endif /* HAPTIC_DIAGNOSTICS_H */
