//********************************
//有wifi配置但连不上时不再自动进入smartconfig，如果需要可以在连接wifi时按住flash键，直到出现Wconfig；
//增加httpupdate，可以直接通过web更新固件；
//按一下重启在Connwifi跳转到时间之前按下flash 8秒松开就能进入smartconfig模式重新配置WiFi
//由于各个品牌的8266运行速度有差异可能出现松开flash无法进入smartconfig模式
//此时迅速的再按下flash 8秒后松开多尝试几次会成功的
//浏览器输入http://8266ip地址/update  可web 网页上传升级固件
//8266ip地址可在串口监视器查看
//尽可能的翻译一些能方便修改的函数使其跟具有适应性
//修改参数注意备份
//********************************

#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeLib.h>
#include <pgmspace.h>
#include <Wire.h>
#include <RtcDS3231.h>

//定义GPIO

#define KEY       D3         // esp8266的D3口和GND接按键，GPIO0 NodeMCU Flash Key flash按键在D3引脚低电平触发
#define LED       16         // NodeMCU板载指示灯
// I2C For ESP8266, these default to SDA = GPIO04(NodeMCU-D2) and SCL = GPIO05(NodeMCU-D1).
#define LEDON LOW //低电平点亮
#define LEDOFF HIGH //高电平熄灭

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

RtcDS3231<TwoWire> Rtc(Wire);

char ssid[30], pass[30], dev_mac[18]; //  WIFI SSID/Password/Mac
static boolean wifiConfiged = false, wifiConnected = false;
#define EEPROM_NUM (sizeof(ssid)+sizeof(pass))
#define LOCATION_WIFI 0x01      //WiFiconfig 写入起始地址
                                
const unsigned long NtpInterval = 21600*1000UL;   //NTP自动校时间隔单位(毫秒)默认值六小时 
unsigned long lastNtpupdate = millis();    //NTP校时标志
boolean rtcstatus = false;   //检测RTC是否有效
boolean ntpstatus = false;   //检测NTP同步状态
unsigned int keys = 0;    //按键检测结果
IPAddress timeServerIP; // 用于存放解析后的NTP server IP；
const char* ntpServerName = "ntp1.aliyun.com";    //NTP服务器域名：尽量不要直接填写IP，


unsigned long ntptime(IPAddress& address){

  const unsigned int localPort = 2395;      // 本地UDP监听端口 
  const unsigned long century = 3155673600UL;        //1900--2000的秒数 不得修改
  const long timeZoneOffset = +28802;     //时区GMT +8  更改个位数可调校时误差 默认值28800
  const int NTP_PACKET_SIZE = 48;     // NTP time stamp is in the first 48 bytes of the message
  byte packetBuffer[NTP_PACKET_SIZE];    //buffer to hold incoming and outgoing packets
  memset(packetBuffer, 0, NTP_PACKET_SIZE);    // set all bytes in the buffer to 0

  WiFiUDP udp;    // A UDP instance to let us send and receive packets over UDP
    udp.begin(localPort);
  delay(500);
  //初始化NTP请求头；
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
    delay(500);      // wait to see if a reply is available
    int cb = udp.parsePacket();
    if (!cb) {
        return 0; //没有收到NTP回包返回0
    }
    else {
        // received a packet, read the data from it
        udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
        
        //the timestamp starts at byte 40 of the received packet and is four bytes,
        // or two words, long. First, esxtract the two words:
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        unsigned long secsSince1900 = highWord << 16 | lowWord;     //得到NTP时间（自1900年以来的秒数）
    return secsSince1900 - century + timeZoneOffset;  //返回NTP同步后的时间
    }
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%04u/%02u/%02u %02u:%02u:%02u"),
            dt.Year(),
            dt.Month(),
            dt.Day(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
}

void scankey()
{
  if (digitalRead(KEY) == 0){
    unsigned int Keytime = 0;
    while(!digitalRead(KEY)){
      Keytime ++;
      delay(10);
    }
    if (Keytime >= 300) keys = 2;
    if (Keytime >= 2 & Keytime < 300) keys = 1;
  }
}

//ssid写入eeprom
 void saveConfig(void){ 
  unsigned char data[EEPROM_NUM];
  unsigned char *pd = data;
  int i;
  for (i=0; i<sizeof(ssid); i++)  {*pd = ssid[i]; pd++;}
  for (i=0; i<sizeof(pass); i++)  {*pd = pass[i]; pd++;}
  for (i = 0; i < EEPROM_NUM; i++) {EEPROM.write(i + LOCATION_WIFI, data[i]);}
  EEPROM.commit();
}

void loadConfig(void) {

  unsigned char data[EEPROM_NUM];
  unsigned char *pd = data;
//  char *ssid_tmp;
//  char *pass_tmp;
  int i;
  for (i = 0; i < EEPROM_NUM; i++) {data[i] = EEPROM.read(i + LOCATION_WIFI);}
  for (i=0; i<sizeof(ssid); i++){ssid[i] = *pd; pd++;}
  for (i=0; i<sizeof(pass); i++){pass[i] = *pd; pd++;}
//  Serial.printf("SSID:%s, PASS:%s\r\n", ssid, pass);
}

void smartConfig(char* ssid_tmp, char* pass_tmp)
{
  WiFi.mode(WIFI_STA);
  Serial.println("\r\nWait for Smartconfig");
  pinMode(LED, OUTPUT);
  WiFi.beginSmartConfig();
  while (1)
  {
    Serial.print("-");
    digitalWrite(LED, LEDON);
    delay(200);
    digitalWrite(LED, LEDOFF);
    delay(200);
    if (WiFi.smartConfigDone())
    {
      Serial.println("SmartConfig Success");
      strcpy(ssid_tmp, WiFi.SSID().c_str());
      strcpy(pass_tmp, WiFi.psk().c_str());
      Serial.printf("SSID:%s\r\n", WiFi.SSID().c_str());
      Serial.printf("PSW:%s\r\n", WiFi.psk().c_str());
    saveConfig();
    pinMode(LED, INPUT);
      break;
    }
  }
}

void connectWifi(char *dev_mac_tmp){
  Serial.print("Connecting to Wifi");
//    WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.printf("ssid: %s,pass: %s\r\n", ssid, pass);
  unsigned int count = 0;
  while ((count < 30 ) && (WiFi.status() != WL_CONNECTED)) {
      delay(500);
      Serial.print(".");
      count ++;
  }
    if(WiFi.status() == WL_CONNECTED){
    Serial.println("Done!");
    Serial.print(WiFi.SSID());
    Serial.println(" Connected!");
    Serial.print("IP addr: ");
    Serial.println(WiFi.localIP());
    Serial.print("Mac addr: ");
    strcpy(dev_mac_tmp, WiFi.macAddress().c_str());
    Serial.println(dev_mac_tmp);  
  }else{
    Serial.println("Fail!");  
  }  
}

void init_rtc(void){
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);   //获取固件编译时间
  if (!Rtc.IsDateTimeValid()) 
  {
    Serial.println("RTC lost confidence in the DateTime!");
    Rtc.SetDateTime(compiled);    //设定RTC时间
  }
  if (!Rtc.GetIsRunning()) Rtc.SetIsRunning(true); //尝试启动Rtc运行
  if (!Rtc.GetIsRunning() || Rtc.LastError() !=0 ) {
    rtcstatus = false;
    setTime(compiled);    //RTC异常后，设定本地时间为编译时间；
  }else{
    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 
    RtcDateTime rtctime = Rtc.GetDateTime();
    setTime(rtctime);   //获取RTC时间，设置本地系统时间
    rtcstatus = true;   //设置RTC有效
    Serial.print("Load RTC Time: "); printDateTime(rtctime);
    Serial.println("\n");
  }
  }

void checklight(){
  
  RtcDateTime Rtctime;
  if (rtcstatus){
    Rtctime = Rtc.GetDateTime();   //读取RTC日期时间数据
  }
  else{
    Rtctime = now();   //RTC不可用时，读取系统日期时间；
  }
   //将系统时间 数据 打印到串口
   Serial.print("Tien: ");
   Serial.print(Rtctime.Year());
   Serial.print(":");
   Serial.print(Rtctime.Month());
   Serial.print(":");
   Serial.print(Rtctime.Day());
   Serial.print(":  ");
   Serial.print(Rtctime.Hour());
   Serial.print(":");
   Serial.print(Rtctime.Minute());
   Serial.print(":");
   Serial.println(Rtctime.Second());
   delay(990);
}
void setup() {
  
  Serial.begin(115200);
  Serial.println();


//初始化
    Serial.println("begin....");

  EEPROM.begin(512);      //初始化EEPROM
  loadConfig();         //读取wifi配置信息
  if(strlen(ssid) != 0 & strlen(pass) != 0){
    Serial.println("ConWiFi");
    connectWifi(dev_mac);       //连接WiFi
    if (strlen(dev_mac) != 0){
      digitalWrite(LED, LEDON);
    }
    wifiConfiged = true;
    Serial.println("wifiConfiged = true");
  }
  else
  {         
    wifiConfiged = false;
    Serial.println("wifiConfiged = false");
  }
    scankey();    //  按键扫描
  Serial.print("scankey=");
  Serial.println(keys);
    if(!wifiConfiged || keys == 2){
    smartConfig(ssid, pass);
  } 

  //初始化RTC时钟，并设定本地系统时间
  init_rtc();

  MDNS.begin(ssid);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);

}

void loop() { 
 // scankey();    //  按键扫描
 //  if ((lastNtpupdate != millis() / NtpInterval) | (keys == 1)) {    //依据计时或按键（短按）进入NTP更新 默认值六小时
   if ((lastNtpupdate != millis() / NtpInterval) ) {    //依据计时或按键（短按）进入NTP更新 默认值六小时
    keys = 0;
    WiFi.hostByName(ntpServerName, timeServerIP);     //get a random server from the pool
    unsigned long curtime = ntptime(timeServerIP);
      if(curtime ==0) {
        Serial.println("NTP update Failure!");
        ntpstatus = false;
      }
      else {
        setTime(curtime);     //写入本地时间
        Rtc.SetDateTime(curtime);   //写入RTC时间
        Serial.print("NTP update Done!");
        ntpstatus = true;
      }
    lastNtpupdate = millis() / NtpInterval;
  }
  
  checklight();
  httpServer.handleClient();
}
