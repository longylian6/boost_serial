# boost_serial
使用boost库实现串口读取并以回调函数方式处理

# 编译运行
直接运行脚本即可
```bash
./buils_script.sh
```

看到如下输出即为成功
```bash
...
[ 33%] Linking CXX executable main
[100%] Built target main
[Serial] Set read callback
[Serial] Open port /dev/ttyUSB1 with baudrate 9600
Received: 01, 02, 03, 04, 05, FF, 
Received: 01, 02, 03, 04, 05, FF, 
[Serial] Close serial port /dev/ttyUSB1
[Serial] Exit receive thread
```
