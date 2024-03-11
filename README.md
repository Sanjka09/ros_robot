d commit нь joystick ээс утга авч rosserial аар sensor topic илгээнэ
ROS serial ажиллуулах заавар:
1.Кодоо шарах
2.Модуль сенсоруудыг тэжээлд залгах
3.In Linux
  sudo chmod 777 /dev/ttyACM0
  roscore
  in another terminal 
  source ~/.bashrc
  rosrun rosserial_pythone serial_node.py /dev/ttyACM0 _baud:=115200
  
