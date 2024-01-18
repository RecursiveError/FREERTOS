# EMBEDDED-API

a simple API project with esp-idf and freeRTOS
the project consists of an API that receives a JSON at the "lcd/" endpoint and writes the value of the "message" field of that JSON to the lcd

This project includes:
- WiFi configuration
- HTTP server configuration
- error handling in the request
- handling errors in JSON
- automatic reconnection on wifi
- update wifi status on lcd

[video](https://youtu.be/paYOd28rvaI)