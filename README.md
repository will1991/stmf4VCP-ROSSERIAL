# stmf4VCP-ROSSERIAL
    这个project是通过USB虚拟串口实现的rosseril_client,通过STM32F407DiscoveryBoard的micro USB 和电脑相连接，
    
    ubuntu识别为/dev/ttyACM*
    
    如果识别不成功，可以尝试断电重启。
    
    更改端口权限：
    
    sudo chmod 666 /dev/ttyACM0
    
    rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
    
## DEMO
    如果连接成功，会有一个/chatter的topic，echo topic输出hello
    rostopic echo /chatter
# 注意：这个Project还在更新，很不稳定！！！
