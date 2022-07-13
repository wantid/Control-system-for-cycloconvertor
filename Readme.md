# Control system for cycloconverter with equivalent phase m = 6

The control system generates control impulses for cycloconverter.

## Description

This repository contains a programm wrote on *C* packed in Keil uVision 5 project. The program code works on the basis of STM32 "F767ZI" debug board.

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

Systick doesn't have any channels, that's why using GPIO (General Purpose Input/Output) is necessary. Using GPIO leads to forming unwabted
programmed delay.

<p align="center"><img src="/GithubMedia/Placeholder.PNG" alt="Unwanted dead time"></p>
<p align="center">fig.3 - Unwanted dead time</p>

### Second implementation

Рассмотрим новую реализацию программы, существенное отличие этого решения заключается в использовании сразу трёх таймеров, 
два из которых это TIM1 и TIM8 – таймеры с расширенным функционалом, а третий TIM2 – таймер общего назначения. 
Использование трёх таймеров обосновано тем, что один является источником синхронизации, а два – таймеры с расширенным функционалом, 
используются вкупе с блоком сравнения для вывода управляющих сигналов.
Использование трёх таймеров в режимах «Master» и «Slave» позволило полностью избавиться от программной задержки в формировании импульсов управления.


## Links

[Telegram] [Youtube]

[Youtube]: https://www.youtube.com/channel/UC3kV-wnqBE3Y2tdtdSrjvGQ
[Telegram]: https://t.me/exeersitus

