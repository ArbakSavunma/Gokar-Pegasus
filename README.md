# **Arbak Savunma Ve Enerji Teknolojileri A.Ş.**

## Gökar Pegasus:
Savunma Sanayi’de İHA ve İKA araçları iki farklı araç olarak üretiliyor. Biz iki aracın dezavantajlarını gidererek bu araçların yaptığı işi tek bir araçla yapmayı hedefliyoruz.

### Donanım Şeması:
#### Hedeflenen Elektronik Tasarım: 
![Pegasus_Elektronik_Şema](https://github.com/ArbakSavunma/Gokar-Pegasus/assets/153490274/5ee254fe-b223-46fa-91d7-7d86e29fe539)

#### 27.05.2024 Elektronik Tasarım: 
![Pegasus_elektronik_şema_pixhawk_microdc](https://github.com/ArbakSavunma/Gokar-Pegasus/assets/153490274/d4b11736-7190-4a0f-8463-080c0c1a88d5)


### Kodların Açıklaması:
Gökar Pegasus aracının çalışması için 5 işlemin gerçekleştirilmesi gerekmektedir.
•	Kumandadan gelen komutların algılanması.
•	IMU sensöründen konum değişiminin ağılanması.
•	PID algoritmasında motor kuvvetlerinin hesaplanması
•	Motorların sürülmesi.
•	LoRa ile istasyon arasında belirli verilerin aktarılması.

Gökar Pegasus algoritması ve test algoritmaları hem Arduino Mega için Arduino IDE'de hem de STM32F4 Discovery kartı için STM32 CUBE IDE platformlarında yazıldı.

Pixhawkla kurulan sistemde kara sürüşünü Arduino Nano ile sağlanmaktadır kara sürüşünün algoritma için de dosyalar _nano.ino kodları bulunmaktadır.
