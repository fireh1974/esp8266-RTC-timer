#DS3231模块网络校时

很久以前用51单片机制作了一个LED时钟，使用了DS3231模块，没有自动校准功能，此模块用来给他附加一个网络校准模块ESP8266
esp8266使用的端口D1、D2、D3
D1->SCL
D2->SDA
D3、GND->flash按钮

//有wifi配置但连不上时不再自动进入smartconfig，如果需要可以在连接wifi时按住flash键，直到出现Wconfig；
//增加httpupdate，可以直接通过web更新固件；
//按一下重启在Connwifi跳转到时间之前按下flash 8秒松开就能进入smartconfig模式重新配置WiFi
//由于各个品牌的8266运行速度有差异可能出现松开flash无法进入smartconfig模式
//此时迅速的再按下flash 8秒后松开多尝试几次会成功的
//浏览器输入http://8266ip地址/update  可web 网页上传升级固件
//8266ip地址可在串口监视器查看
