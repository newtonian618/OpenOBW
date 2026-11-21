本项目用STM32F103C8T6控制BMI088采集数据，并通过RS485传输数据。电路图见OpenOBW.pdf，IO分配如下：
    - PA1：VINSEN，供电电压采集，33k上拉，3k下拉分压
    - PA2：TEMP，温度采集，B值3450的10k热敏电阻上拉10k分压
    - PA3：nCSBG, BMI088的陀螺仪片选
    - PA5：SCK, BMI088的SPI时钟
    - PA6：MISO, BMI088的SPI数据输入
    - PA7：MOSI, BMI088的SPI数据输出
    - PA9：LED2，LED指示灯
    - PA10：LED1，LED指示灯
    - PB0：INTG1，BMI088的陀螺仪中断1
    - PB1：INTG2，BMI088的陀螺仪中断2
    - PB10：RXD3, RS485接收数据
    - PB11：TXD3, RS485发送数据
    - PB12：nCSBA，BMI088的加速度计片选
    - PB13：INTA1，BMI088的加速度计中断1
    - PB14：INTA2，BMI088的加速度计中断2

实现以下功能：
1. 开机后LED1点亮；
2. BMI088初始化，配置SPI和中断，如果配置失败，LED2连续闪烁，程序终止；
3. BMI088初始化为50Hz采样率，MCU通过中断读取数据，每100ms数据进行一次平均并保存在缓冲区等待读取指令；缓冲区只保存最新的一个数据；
4. 采集温度无需用vrefint，直接用ADC采样即可；采集电压需要用vrefint进行校准。
5. 命令线程等待命令，根据命令进行相应操作。命令与E:\project\BTASystem\BTA-6_fmw\protocol.txt中的协议一致。
6. 程序中defaultTask负责采集数据并保存在缓冲区，其他任务负责处理命令和发送数据；commandTask负责处理命令。；evenTask负责异常处理，包括BMI088异常，电压低于10V或高于36V LED1闪烁；温度低于-20℃或高于75℃ LED1闪烁。
7. 程序使用英文注释和变量名。
8. 完成后写一份README.md，说明项目结构、通信协议、命令示例以及如何编译和烧录程序。
