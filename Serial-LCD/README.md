
# LCD SERIAL


freeRTOS + esp-idf learning project to control an alphanumeric lcd display using UART

the project consists of 3 tasks:

"read_uart" reads the information from the serial to \n or \r.
returns whether the operation was a success or not, sends the information through a queue to "terminal_lcd"

"terminal_lcd" receives a message from the read_serial queue, and checks if it is a valid command for the lcd
(a valid command consists of a value inside < > followed by an argument inside ( ))
both are mandatory, sends the result to read_uart and sends the command to "lcd_server"

"lcd_server" uses the external library "UniversalLCD" to control the LCD using an I2C interface
UniversalLCD requires a delay of microseconds to work
which cannot be done with freeRTOS tiks, since the default is 1Tick/1Khz
the delay It is done with a gptimer from esp-idf to generate a delay (which does not block the task)

"lcd_server" receives a command and executes the task (this example only has 7 commands:
<write>(args), <clear>(), <setline>(line col), <cpp>(), <curso>(state), <display>(state) and <moved>(dir times)
*<cpp> show cpp logo