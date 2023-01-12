# ME331项目 基于webots仿真串联\并联四足机械狗

本项目包含webots世界文件(串联并联各一个)以及webots controller文件(串联并联各一个）

实现了串联、并联正逆运动学计算、trot walk步伐控制、基于trot的原地转弯、固定半径转弯、斜坡平衡（仅并联机体）
用keyboard包实现了键盘控制移动
将世界文件用webots直接读取后 Robots选择使用<extern>控制器 在外部运行controller.py即可使用键盘控制移动
!(https://github.com/Akito-UzukiP/Parallel-quadruped-robot-dog/blob/main/pictures/1.png)
