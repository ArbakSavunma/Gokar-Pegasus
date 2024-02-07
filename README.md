# **Arbak Savunma Ve Enerji Teknolojileri A.Ş.**

## Gökar Pegasus:
Savunma Sanayi’de İHA ve İKA araçları iki farklı araç olarak üretiliyor. Biz iki aracın dezavantajlarını gidererek bu araçların yaptığı işi tek bir araçla yapmayı hedefliyoruz.

### Donanım Şeması:
![Pegasus_Elektronik_Şema](https://github.com/ArbakSavunma/Gokar-Pegasus/assets/153490274/abd89633-17f8-4963-8be2-2b88452eff4e)

### Kodların Açıklaması:
Gökar Pegasus aracının çalışması için 5 işlemin gerçekleştirilmesi gerekmektedir.
•	Kumandadan gelen komutların algılanması.
•	IMU sensöründen konum değişiminin ağılanması.
•	PID algoritmasında motor kuvvetlerinin hesaplanması
•	Motorların sürülmesi.
•	LoRa ile istasyon arasında belirli verilerin aktarılması.

Araç algoritması ve test algoritmaları hem Arduino Mega için Arduino IDE'de hem de STM32F4 Discovery kartı için STM32 CUBE IDE platformlarında yazıldı.

#### Kumandadan Gelen Komutların Algılanması:
https://www.katranji.com/tocimages/files/390729-235499.pdf
Elimizde bulunan Mikrozone MC6C kumanda vericisi ve MC6RE alıcısını kontrol kumandası olarak kullanacağız. İlgili dökümanının linkini yukarıda verdim. STM32F4 ün Timer 1 ve Timer 3 zamanlayıcı pinlerinin 6 tanesini alıcının 6 kanalından PWM sinyalini okumak için Input Directure modunda interrupt çalışmasında okundu. Test kodu hazırlandıktan sonra ana sürüş algoritmasına alındı. Yararlandığım örnek kod linkini aşağıda verdim. 
https://github.com/EnesTayfun/signal_capture_stm32_nucleol476rg/tree/main
Arduino IDE:
*transmitter_read
STM32 CUBE IDE:
*mikrozone_readData

#### Fırçasız DC Motorların Sürülmesi:
Fırçasız dc motorların kontrolü hangi sürücü (ESC) veya mikrodenetleyicinin kullanırsak kullanalım sabit bir mantık içerir. Servo motorların kontrolündeki gibi 20ms lik periyoda sahip darbe genişlik sinyallerinin (PWM) 1ms – 2ms arasındaki zaman dilimlerini maksimum ve minimum dutycycle olarak kullanabiliriz. Timer 2, Timer 4 ve Timer 8 zamanlayıcı pinlerini; pervane motorları, tekerlek motorları ve eklem servo motorlarının kontrolü için kullanıldı. Test edildikten sonra ana sürüş koduna eklendi. Çalışma için yararlandığım kaynağın linkini aşağıya bıraktım.
https://controllerstech.com/how-to-interface-bldc-motor-with-stm32/
Arduino IDE:
*mootor_kalibre
STM32 CUBE IDE:
*pwm_motorsurme3

#### IMU sensöründen konum değişiminin ağılanması:
Gökar Pegasus’un PID kontrol algoritmasında ana geri dönüt sinyali IMU sensörüdür. “Testlerde ağırlıklı MPU6050 IMU sensörü kullanıldı. 3 eksen ivme ölçer, 3 eksen de gyrometreden veri aktaran MPU6050 ile konum değişimini hesaplıyoruz. 6 eksen veriden açı değişimlerini hesaplamak için algoritmaya bazı filtreler uygulanmaktadır. Düşük geçirgen filtre, yüksek geçirgen filtre, madwick fitresi ve kalman filtreleri arasından düşük geçirgen filtreyi seçtik. Sensör kullanımı için yararlandığım kaynakların linklerini aşağıya bıraktım.
https://github.com/berkaysaka/ArduinoQuadcopterFlightController/tree/master/lessons/3_reading_imu
https://github.com/EnesTayfun/mpu6050_library
Arduino IDE:
*IMU
STM32 CUBE IDE:
*IMU_MPU6050

#### PID algoritmasında motor kuvvetlerinin hesaplanması
Pegasus aracının kullanıcı komutlarını göre stabil ve doğru uygulaması için her bir motorun dönüş hızı PID algoritması ile kontrol edilmektedir. Kumandadan gelen komutlar ile IMU sensöründen gelen anlık konum verisinin farkını hata sinyali olarak kullanılmakta. Hata verisini PID algoritması kullanılarak her bir motor için ayrı PWM sinyali hesaplatıyoruz. PID sabitlerinin belirlenmesi de aracın ağırlık dengesi ve montajı tamamlandıktan sonra deneme yanılma yolu ile test edilip belirlenmektedir. 
PID algoritması tamamlandıktan sonra ana sürüş koduna eklendi ve farazi PID sabitleri ile test edilmektedir. Araç mekaniği ve montajı bittikten sonra da PID sabitlerinin tespiti için testler yapılacaktır. Yararlandığım kaynaklardan birini aşağıya bıraktım.
https://idus.us.es/bitstream/handle/11441/108818/TFM-1874-LOPEZ%20FLORES.pdf?sequence=1&isAllowed=y

#### LoRa ile istasyon arasında belirli verilerin aktarılması.
Projenin ikinci aşaması olan otonom geri dönme için kontrol istasyonunda kurulu ROS sistemi ile araç arasındaki iletişimi LoRa modülleri ile gerçekleştireceğiz. Çift yönlü olacak olan bu telemetri, aracın konum bilgilerini haritalama için kontrol istasyonuna gönderilecek. Haritalamanın sonucunda belirlenen rotada gidebilmesi için hesaplanan hareketler de kontrol istasyonundan araca aktarılacaktır. Lora iletişim algoritması daha Ar-Ge aşamasındadır ama algoritma tasarımı için kaynak taraması yapılmıştır.

#### Ana Algaritmalar:
Projenin testleri için hem Arduino Mega hem de STM32F4 Discovery kartlarına algoritma hazırlanmaktadır. Test edilen her algoritma adım adım ana algoritmalara entegre edilip versiyon atlatılmaktadır.
Arduino IDE:
*pegasus_multi_driver
*pegasus_multi_driver_model2
*pegasus_multi_driver_model3
STM32 CUBE IDE:
*pegasus_test_vol1
*pegasus_test_vol2
*pegasus_test_vol3
*pegasus_test_vol4
