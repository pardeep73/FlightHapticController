# Stage 4 diagnostic commands. Run from the extracted project root with the
# matching ELF loaded. Reads variables; no target-data assignments or HAL calls.
# First start Wokwi with "Start Simulator and Wait for Debugger".

set pagination off
set confirm off
set logging file stage-4-write.txt
set logging overwrite on
set logging redirect off
set logging enabled on

# This source prefix is embedded in the unchanged packaged ELF's debug data.
set substitute-path /tmp/flight-haptic-stage2-a3gplb91/project .
set $stage4_hit = 0

echo Connecting to the local Wokwi debugger. A fresh paused startup is required.\n
target remote 127.0.0.1:3333

# The write's return has been stored at this point; GPIO fallback has not run.
break EndHardwareTrace if haptic_app_status.register_address == 0x6b
commands
  silent
  set $stage4_hit = 1
  printf "STAGE4 op=WRITE reg=0x%02x HAL=%u err=0x%08x\n", (unsigned int)haptic_app_status.register_address, (unsigned int)haptic_app_status.hal_status, (unsigned int)sensor_i2c->ErrorCode
  printf "TRACE flag=0x%08x ISR=0x%08x CR2=0x%08x\n", (unsigned int)hw_trace_flag, (unsigned int)hw_trace_isr, (unsigned int)hw_trace_cr2
  printf "LIVE ISR=0x%08x CR1=0x%08x CR2=0x%08x TIMINGR=0x%08x\n", (unsigned int)sensor_i2c->Instance->ISR, (unsigned int)sensor_i2c->Instance->CR1, (unsigned int)sensor_i2c->Instance->CR2, (unsigned int)sensor_i2c->Instance->TIMINGR
  printf "XFER remaining=%u size=%u state=0x%08x mode=0x%08x\n", (unsigned int)sensor_i2c->XferCount, (unsigned int)sensor_i2c->XferSize, (unsigned int)sensor_i2c->State, (unsigned int)sensor_i2c->Mode
  printf "CONTEXT who=0x%02x split=%u serr=0x%08x software_bus=%u\n", (unsigned int)haptic_app_status.identity, (unsigned int)haptic_app_status.split_read_stage, (unsigned int)haptic_app_status.split_read_error, (unsigned int)haptic_app_status.software_i2c
  bt 5
end

continue

if $stage4_hit
  echo Capture complete. The CPU is paused before GPIO fallback.\n
  echo Preserve stage-4-write.txt, then stop Wokwi and quit GDB.\n
else
  echo The wake-write breakpoint was not reached. This is not a completed capture.\n
end

set logging enabled off
