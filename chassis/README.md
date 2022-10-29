# RM-Infantry-Chassis v1.1

### 工程统一
1. cubemx配置的STM32F427主频设置为180MHz（使用HSE时钟）
2. `menuconfig` 下设置`(1000) Tick frequency,Hz`
3. 不使用虚拟文件系统了   
4. can使用硬件过滤表 

### 目前底盘完成情况

- 能用遥控器控制或云台通信控制
- 实现跟随云台与不跟随（包含小陀螺）两种模式
- 但未调试过IMU和裁判系统

### 所做工作
- 主要是在原有步兵底盘代码基础做了整理  
- 移植到rtt 4.0.2版本上  
- 用 `scons --dist` 做出发行版
- 整理了工程目录结构

### 使用注意点
#### can通信问题

由于rtt 4.0.2版本自身底层驱动问题，当Cubemx设置的STM32F427的主频是168MHz时，can的通信是有问题的。  
详情参考 https://club.rt-thread.org/ask/question/424684.html

使用168M频率时，会卡在以下代码处
```
rt_completion_wait(&(tx_tosnd->completion),RT_WAITING_FOREVER);
```
**解决方法**：设置主频为180MHz  

#### Sconscript
目前仅更改过原生bsp的applications目录下的Sconscript文件。  
其他Sconscript and SConstruct文件没有做过任何更改。

#### Kconfig
- 目前暂时将底盘种类选取的宏（麦轮or全向轮）放在`Choose Car model Config`菜单下进行管理  
- 以及软件监视器等宏放在`Onboard Peripheral Drivers`菜单下进行管理。  
- 修改`\RM-Infantry-Chassis\Kconfig`文件 增加一句`source "applications/Kconfig"`

#### iar
使用iar IDE可能存在问题