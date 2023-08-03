Import("env")

env.Execute("tools/pioasm src/memory_sm0_clock.pio src/memory_sm0_clock.pio.h")
env.Execute("tools/pioasm src/memory_sm1_address.pio src/memory_sm1_address.pio.h")