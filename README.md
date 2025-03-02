# ArduinoFFBJoystick
(日本語)
【概要】

War Thunderゲーム用の2軸フォースフィードバックジョイスティックです。  
Arduino LeonaldとMegaの２つのコントローラを使います。  
Arduino LeonaldをHIDデバイスととして設定し、モータ制御はMegaが担当します。  
Megaを利用する理由は、PWM用のタイマが３個以上利用できるので将来Z軸にも対応するためです。  
2軸だけであれば、安価なUnoでも利用できると思います。  



【コンパイル】  
Leonald側（マスタ）  
以下のアドレスを開き、CodeからDownload.zipを選んでダウンロードします。  
https://github.com/GNZ-3/ArduinoFFBJoystick  
ご自身のArduinoフォルダ(Documents\Arduino\)に展開してください。  
ライブラリマネージャから以下をインストールします。  
digitalWriteFast  
DynamicHIDフォルダをLibraryフォルダ（Documents\Arduino\Library）にコピーします。  

Mega側（スレーブ）  
以下のアドレスを開き、CodeからDownload.zipを選んでダウンロードします。  
https://github.com/GNZ-3/MegaSlave  
ご自身のArduinoフォルダ(Documents\Arduino\)に展開してください。  
ライブラリマネージャから以下をインストールします。  
AVR_PWM  
または、以下からダウンロードしてライブラリをインストールします  
https://github.com/khoih-prog/AVR_PWM  

【部品】  
Arduino Leonald x1  
Arduino Mega2560 x1  
AS5600 position sensor x2  
BTS7960 43A H bridge Motor Driver x2  
Xinda Xd3D30-12GnDC Geared Motor 減速比30、110RPM x2  
（試験のため私の場合、１つはXD2D15-12GN-21Sを利用しています）  



【接続図】  
![Schema](https://github.com/GNZ-3/ArduinoFFBJoystick/blob/main/Misc/ArduiniFFBJoystick.png)


謝辞  
York Lawさんの偉大なFFBライブラリです。これがなかったら完成できませんでした。  
https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary  
Khoi HoangさんのPWMライブラリです。PWM出力を10kHzに設定するとノイズが低減できます  
https://github.com/khoih-prog/AVR_PWM  

# ArduinoFFBJoystick 
(English)
[Summary]
2 axis Force Feedback joystick for War Thunder games.
You will need both Arduino Leonald and Mega.
Leonald act as HID device, mega act as a motor controll.
The reason to use mega is that mega has more than 3 timers which can extend to 3 axis in the future.
If you only needs 2 axis FFB, uno can be used.

[Compile]
Leonald for Master
Open link below and then Code -> Download.ZIP to dowonload it.
https://github.com/GNZ-3/ArduinoFFBJoystick
Extract it to your own Arduino folder(Documents\Arduino\)
Install below from Library Manager in IDE
digitalWriteFast
Copy DynamicHID folder into Library folder (Documents\Arduino\Library).

Mega for Slave
Open link below and then Code -> Download.ZIP to dowonload it.
https://github.com/GNZ-3/MegaSlave
Extract it to your own Arduino folder(Documents\Arduino\)
Install below from Library Manager in IDE
AVR_PWM

[BOM]
Arduino Leonald x1
Arduino Mega2560 x1
AS5600 position sensor x2
BTS7960 43A H bridge Motor Driver x2
Xinda Xd3D30-12GnDC Geared Motor gear ratio 30, 110RPM x2
( For testing purpose, I am using  XD2D15-12GN-21S as Y axis instead.）

[Special thanks]
York Law's greate FFB library.
https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary
Khoi Hoang's PWM library.
https://github.com/khoih-prog/AVR_PWM
