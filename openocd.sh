openocd -f interface/stlink-v2.cfg -c "adapter_khz 1000; transport select hla_swd" -f target/stm32h7x.cfg
