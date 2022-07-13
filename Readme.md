# Control system for cycloconverter with equivalent phase m = 6

The control system generates control impulses for cycloconverter.

## Description

This repository contains a programm wrote on *C* packed in Keil uVision 5 project. The program code works on the basis of STM32 «F767ZI» debug board.

## Development

First of all, for creating a program that forms control impulses, it is necessary to know count and form of the impulses. 
In my case, there are six impulses 3,3ms length, because of cycloconverter consist of six transistors and it works on 50 Hz. 

<p align="center"><img src="/GithubMedia/Placeholder.PNG" alt="Control impulses"></p>
<p align="center">fig.1 - Control impulses</p>

### First implementation

Creating control system on Systick is a simple issue, that's why first implementation was created using it. Simplicity of a 
system timer illustrates its structure, it has only 4 registers but my program uses only 2.

<p align="center"><img src="/GithubMedia/Placeholder.PNG" alt="Systick structure"></p>
<p align="center">fig.2 - Systick structure</p>

Systick doesn't have any channels, that's why using GPIO (General Purpose Input/Output) is necessary. Using GPIO leads to forming unwanted
programmed delay.

<p align="center"><img src="/GithubMedia/Placeholder.PNG" alt="Unwanted dead time"></p>
<p align="center">fig.3 - Unwanted dead time</p>

### Second implementation

Instead of using systick, the final solution uses three timers: TIM1, TIM8 - advanced control timers and TIM2 - general purpose timer.
These timers were used in modes «Master» and «Slave» for synchronization. This solution solved the problem with unwanted delay. 
## Links

[Telegram] [Youtube]

[Youtube]: https://www.youtube.com/channel/UC3kV-wnqBE3Y2tdtdSrjvGQ
[Telegram]: https://t.me/exeersitus

