# Control system for cycloconverter with equivalent phase m = 6

The control system generates control impulses for cycloconverter.

## Description

This repository contains a programm wrote on *C* packed Keil uVision 5 project. The program code works on the basis of STM32 "F767ZI" debug board.

## Development

Прежде всего, для синтеза программы реализующей формирование управляющих импульсов необходимо задаться сигналами 
управления и их количеством. В исследуемом преобразователе реализуется управление на базе шести транзисторов, сигналы, 
подводимые к ним сигналы в модели, получены ранее в прошлой главе. НПЧ-6 работает на синхронной частоте равной 50 Гц, 
на ключи подаются импульсы длиной 3,3 мс. 

<p align="center"><img src="/GithubMedia/Placeholder.PNG" alt="Control impulses"></p>
<p align="center">fig.1 - Control impulses</p>

### First implementation

Реализация системы управления на базе системного таймера по своей имплементации является довольно простой, 
поэтому изначально программа, формирующая импульсы управления, была написана именно на нём. Простоту системного 
таймера наглядно иллюстрирует его структура, поскольку у этого таймера всего четыре регистра. При этом используются лишь два.

<p align="center"><img src="/GithubMedia/Placeholder.PNG" alt="Systick structure"></p>
<p align="center">fig.2 - Systick structure</p>

Так как системный таймер не имеет собственных каналов, привязанных с к выводам микроконтроллера, установка открытого и закрытого 
состояния вывода с микроконтроллера устанавливалась не напрямую, а через регистр GPIO. По этой причине на выходных сигналах 
появлялась вынужденная программная задержка. 

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

