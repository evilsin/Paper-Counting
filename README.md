# Paper-Counting
纸张计数显示装置
设计内容摘要
本系统主要由纸张固定平台、传感器模块、处理器模块及显示模块组成。固
定平台由精密手动平口钳和 25KG 舵机组成，传感器模块使用 FDC2214 电容传感
器和二线制平行极板电极，处理器模块采用 STM32F1 单片机。本系统通过 FDC2214
采集极板间传感信号，STM32F103 通过 IIC 接口读取测量值，并对信号进行卡尔
曼滤波和数据归一化处理，提高了采集数据的精确度。最后对采集数据进行分析
建模，得出可自校准的回归模型。实验结果表明，该系统可对 50 张以内的纸张
进行精确检测。

预期实现目标定位
纸张计数显示装置可以在5s内准确的测量出极板间30张以内的纸张数同时
可以发出蜂鸣并在 OLED 屏显示纸张数，对 60 张以上的测量计数可以做到将误
差控制在一张以内，并且在不同的环境中可以进行“自校准”，测量的同时可以
检测极板间是否出现短路情况。

软件部分通过 STM32F103 单片机实现，系统工作流程图如图 4.1 所示。纸张
数量检测系统主要由系统初始化、自校准、纸张检测和循环检测四个部分组成。
上电启动后，自动进行初始化配置，显示欢迎界面。当用户放置好纸张并按下自 校准按键时，系统采集 FDC2214 传感器信号并由此校正回归系数。直到校准次数
达到设定值后，系统开始纸张检测，可以检测纸张数量并在 OLED 屏幕显示。
（1）卡尔曼滤波
利用卡尔曼滤波器对 FDC2214 传感器采集信号进行处理，保证信号稳定性。
（2）数据归一化
对输入数据进行标准化处理，使数据映射在 0~1 范围内。
（3）分段回归
通过理论推导与实验分析，采用分段回归方法完成纸张数量检测系统建模。

设计分析软件环境
（1）数据处理环境：MATLAB
（2）单片机开发环境：Keil4、Keil5
（3）硬件电路设计环境：Altium Designer15.0
仪器设备硬件平台
（1）FDC2214 抗 EMI 28 位电容数字转换器
（2）STM32F103ZET6 芯片
（3）OLED 屏幕

![image](https://user-images.githubusercontent.com/30195788/158121718-1f63b099-c692-406e-9b06-e2d51a60e625.png)
![image](https://user-images.githubusercontent.com/30195788/158121784-b22ac1e4-41da-4ee8-8786-f1964a7624b5.png)

总结
采用 FDC2214 传感器的纸张计数显示装置受纸张空隙、电磁干扰、
环境温湿度等不可控因素影响较大，基于此，本系统分别在硬件装置和软件算法
上对信号波动进行优化，硬件上舵机固定扭矩对纸张进行固定，并采用屏蔽线减
少外界干扰。软件算法上，对采集数据进行卡尔曼滤波和数据归一化处理，使得
信号稳定，数据精确性极大提升。实验结果显示，本系统能够适应不同环境，准
确检测 50 张以内的纸张。但仍然存在人工操作下难以克服的随机误差，下一步，
可以设计恒力矩舵机自动固定装置，保证每次测量时纸张受力一致，将得到更加
稳定精确的纸张计数装置
