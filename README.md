# Smart Fan
<img src="https://github.com/user-attachments/assets/d59d44d8-bc2c-4038-80a0-ccd6c28630e3"  width="720px">

## ✏️[프로젝트 개요]

### 프로젝트명 : Smart Fan (스마트 선풍기)

**<사용자 맞춤 온도 감응형 선풍기>**

일반적인 선풍기는 사용자가 원하는 동작 온도를 직접 설정할 수 없으며, 풍량 조절이나 타이머 기능만으로는 환경 변화에 유동적으로 대응하기 어렵습니다. 사용자가 항상 수동으로 조절해야 하고, 실내 온도 변화에 따라 반복적으로 조작해야 하는 불편함이 존재합니다. 특히 수면 중인 사용자나, 스스로 체온을 조절하기 어려운 영유아, 반려동물 등에게는 정밀한 온도 기반 제어가 필요할 수 있습니다.

이러한 문제를 해결하기 위해, 사용자가 설정한 희망 동작 온도에 따라 자동으로 동작하는 스마트 선풍기를 제안하고자 합니다.<br>

## 📙[프로젝트 기능]

### 온도 감지 및 표시
온도센서를 통해 실시간으로 주변 환경의 온도를 감지하고 표시합니다.

### 희망 동작 온도 설정
사용자가 Up, Down 버튼으로 몇 도 이상일 때 선풍기가 자동으로 작동할지를 설정할 수 있습니다.<br>
(동작 조건: 실제 온도 >= 희망 동작 온도, default: 30°C)

### 절전 모드
절전 스위치를 키면 실제 온도가 희망 동작 온도에 도달하여도 선풍기가 동작하지 않습니다.

## 🎬[동작 화면]

|                시작화면                |
|:-------------------------------------:|
|![1](https://github.com/user-attachments/assets/c2b5809a-cfd6-4323-8577-3a01d4aa5f21)|
|전원이 들어오면 가장 먼저 보여지는 화면|

|            실시간 온도 감지            |
|:-------------------------------------:|
|![2](https://github.com/user-attachments/assets/9e9fa638-337e-44f6-a810-c437ae49aec3)|
|온도 센서에 감지된 온도가 실시간으로 변화합니다.|

|           희망 동작 온도 설정           |
|:-------------------------------------:|
|![3](https://github.com/user-attachments/assets/8ea21a61-ddbb-4df5-b9e3-d956cf695ce3)|
|실제 온도가 희망 동작 온도 보다 높으면 LED와 선풍기가 켜집니다.|

|                절전모드                |
|:-------------------------------------:|
![4](https://github.com/user-attachments/assets/4656ca40-d714-4f56-98d8-991ceaa29e01)
|실제 온도가 희망 동작 온도 보다 높아도 선풍기가 켜지지 않습니다.|

## 🎞️[시연 영상 링크]
<a href="https://youtu.be/B4J0zdUsopM">유튜브 링크</a>

## 🛠[프로젝트 구조]

<img src="https://github.com/user-attachments/assets/70bb24df-b01f-42f5-a10e-5403f2f91ff3">
<img src="https://github.com/user-attachments/assets/b4f014b3-8dc8-4c7e-aed3-5eac4ab0418c">

**개발 언어** : C(HAL 라이브러리)<br>
**개발 도구** : STM32CubeIDE, Visual Paradigm<br>
**개발 보드** : STM32F103C8T6

## 🔍[기대 효과]

**<냉방 가전의 새로운 제어 방법 창출>**

기존의 풍량 조절이나 타이머 기능을 넘어서, 한층 진화된 제어 기능을 추가함으로써 사용자 선택의 폭이 더욱 넓어질 것으로 보입니다.
