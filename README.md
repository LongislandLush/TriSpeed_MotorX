# TriSpeed_MotorX
STM32-based 3-speed DC motor controller with PWM, UART, and state logic

TriSpeed MotorX 是一個基於 STM32F446RE Nucleo 開發板的小型馬達控制專案，
實現了按鈕切換三段轉速、OLED即時顯示當前狀態，並透過 UART 通訊將速度資訊同步給 Arduino，控制 RGB LED 顏色變化。

# 系統架構
## STM32 X Arduino X UART X PWM X OLED 系統架構圖
![TriSpeed_MotorX_系統架構圖](images/TriSpeed_MotorX_系統架構圖.png)


## Normal Mode 流程圖
![Normal Mode Flowchart](images/Normal_Mode_Flowchart.png)


## Setting Mode 流程圖
![Setting Mode Flowchart](images/Setting_Mode_Flowchart.png)


# 使用方式

1. 開啟STM32 Cube IDE, 可以參考 `STM32` 資料夾裡的main.c, .ioc檔設定並燒錄到Nucleo F446RE.
   
2. 開啟Arduino IDE,  可以參考 `Arduino` 資料夾裡的.ino檔並燒錄到Arduino Uno R3.
   
3. 接線參考images資料夾裡的接線檔案, 並且要注意板子與各模組需要在麵包板上共地.

4. STM32是 +3.3V, Arduino是+5V, 所以STM32 TX可以直接插在Arduino RX上, 但是Arduino TX要經過分壓電路, 將電壓降成+3.3V左右再接到STM32 RX上.

5. L298N Motor Driver Module有另外接上6-AA電池盒單獨對直流馬達供電, 模組, 電池盒 & 麵包板的負極也需要共地.
  
6. 按鈕有做防彈跳電路, LED也有接上限流電阻, 可以參考images資料夾裡的接線檔案.
   
7. 確認硬體接線和韌體都燒錄OK, 可以在Normal Mode & Setting Mode下測試按鈕1, 按鈕2 & UART輸入指令下直流馬達, LED & OLED是否如預期.
  
8. 使用 UART 在Setting Mode 下, 可以用鍵盤傳藉由終端機傳送以下指令 (`0`, `1`, `2`, `3`) 來切換直流馬達速度.

# 功能說明

按鈕1：短按切換弱速 ➔ 中速 ➔ 高速循環

按鈕2：短按重置為弱速、長按進入設定模式（停止馬達）

設定模式下，透過終端機（UART）輸入數字指令設定馬達轉速

OLED螢幕顯示當前模式（Normal / Setting）與轉速狀態（Stop / Weak / Mid / High）

UART同步傳送狀態給Arduino，改變RGB燈顏色


# 使用技術與工具

**IDE**: 

1. STM32CubeIDE
   
2. Arduino IDE

**Hardware**: 

1. STM32F446RE Nucleo Board x1
 
2. Arduino Uno x1
 
3. OLED (SSD1306) x1
 
4. L298N Motor Driver Module x1
 
5. DC Motor(F130 Type) x1
 
6. RGB LED Module(HW-479) x1
 
7. LED(White) x1
 
8. Push Button x2
 
9. 6xAA Battery Holder x1

10. Resistors:
    – 330 Ω ×1
    – 1 kΩ ×1
    – 2 kΩ ×1
    – 10 kΩ ×2

11. Capacitors:
    – 104 Ceramic Capacitor (0.1 µF) ×2

12. Breadboard ×1

13. Jumper Wires: Male-to-Male, Male-to-Female (Depending on connection)

***所有零件皆為常見開發模組，適合初學者搭建與測試***

**Knowledge:**

1. PWM 控制直流馬達轉速
   
2. 按鈕消除彈跳
 
3. UART通訊
 
4. C Programming Language

# 開發過程重點

1. 實作軟體與電路層級的去彈跳機制，有效避免按鈕誤觸。

2. 導入狀態機（State Machine）架構，實現模式與速度的穩定切換。

3. 透過分壓電路解決不同開發板間 UART 通訊的電壓相容問題。

4. 使用 OLED 即時顯示當前模式與馬達狀態，並可透過終端機進行遠端操作與控制。

# 成果展示


# 授權
***本專案以 MIT License 授權開源使用。***
