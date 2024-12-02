基于ESPIDFV5.1.4的adc_continuous库改写而来
取消了DMA中断，改为阻塞读取，降低高刷新率下的中断开销。目前1.5MHz采样率下3通道读取可以达到3.2uS,300KHz左右 单通道极限速度约613khz

注意该参数
#define INTERNAL_BUF_NUM      5
内部缓冲区增加的倍数 过小会读取有问题，缓冲区内数据容易漏或乱序，疑似DMA重启速度慢。
反复尝试未能找到原因，所以我这个库也加大缓冲区，从中提取有用数据


Based on the adc_continuous library rewritten from ESPIDF V5.1.4, the DMA interrupt has been removed and replaced with blocking reads, reducing interrupt overhead at high refresh rates. Currently, at a sampling rate of 1.5MHz, reading three channels can achieve 3.2µs, and the single-channel limit speed is about 613kHz at around 300kHz.

Note this parameter:
#define INTERNAL_BUF_NUM      5
This defines the multiple increase of the internal buffer. If it's too small, reading issues may occur, resulting in data loss or out-of-order sequences within the buffer, likely due to slow DMA restart speeds. I have not been able to find the cause despite repeated attempts, so I increased the buffer size in this library to extract useful data.

