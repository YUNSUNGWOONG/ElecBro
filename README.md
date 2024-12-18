# AGRIBOT(어그리봇)
<p align="center">
  <br>
 	<img src="https://github.com/user-attachments/assets/67315750-e5d9-4546-bf5f-3e2b63acf252" alt="elebro_logo" style="zoom:50%;" />
  <br>
</p>


### 2024임베디드SW경진대회(webOS부문) - ELECBRO팀



## 1. Getting Started(시작방법)

These instructions will give you a copy of the project up and running on
your local machine for development and testing purposes. See deployment
for notes on deploying the project on a live system.

## 2. Prerequisites(요구사항)

아래와 같은 소프트웨어 개발환경을 필요로 합니다.
- [VScode](https://www.example.com)
- [Example 2](https://www.example.com)

또한, 다음과 같은 하드웨어가 필요합니다.

- RaspberryPi4
- HaviBot
- ConveyorBelt
- HueBot





## 3. Installing

### 3.1 webOS 세팅방법

NodeJS 설치 후, 아래 절차를 따라줍니다.

```javascript
# 요구사항 패키지 설치
npm install
# node서버 실행
npm run dev
```



### 3.2 HaviBOT 세팅방법

- 터틀봇 패키지 설치



- Havibot에 있는 

### 3.3 HueBOT 세팅방법

0. 패키지 설치전 필요한 것들을 다운받아줍니다.
   ```bash
   sudo apt update
   sudo apt full-upgrade
   ```

   

1. 아래 명령어를 통해 라즈베리파이에 mini-forge를 셋업하여 미니콘다 셋업파일을 설치해줍니다.
   ```bash
   wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
   ```

   

2. 설치파일을 실행해줍니다.

   - (sudo는 사용하지 않습니다.)

   ```bash
   bash Miniforge3-Linux-aarch64.sh
   ```

   - 모든건 yes로 대답하고, 기본값을 사용해준다.

   ```bash
   Miniforge3 will now be installed into this location:
   /home/pi/miniforge3
   
     - Press ENTER to confirm the location
     - Press CTRL-C to abort the installation
     - Or specify a different location below
   
   [/home/pi/miniforge3] >>>
   ```

   - 정상 수행되면, 마지막 질문에 대답하면 됩니다.

   ```bash
   Transaction finished
   installation finished.
   Do you wish the installer to initialize Miniforge3
   by running conda init? [yes/no]
   [no] >>> yes
   ```

   - 설치가 완료되었다면 아래 절차를 이행해줍니다.

   ```bash
   conda config --set auto_activate_base false
   export PATH=$PATH:/home/pi/miniforge3/bin
   ```

   (필자는 사용자명을 pi로 두었기에 저렇게 환경변수 설정을 해주었으나, 사용자명에 맞게 위 사용자명 부분을 수정해서 사용하면 된다)

   - 이후 재부팅을 해준다.

   ```bash
   sudo reboot
   ```

   (그러면 이제 터미널을 켤때 앞에 (base)가 붙은것을 확인할 수 있는데, 이는 정상적으로 아나콘다 프롬프트가 적용되었다는 뜻입니다.)

3. 이후  `elecbro.yml` 파일을 활용하여 가상환경을 만들어줍니다.
   ```bash
   conda env create --file ./envs/elecbro.yaml
   ```

   

4. 만들어진 가상환경을 실행시켜줍니다.

   ```bash
   conda activate elecbro
   ```

   



## Contributing

- 윤성웅(전자공학부 20학번)
- 김현근(전기공학부 19학번)
- 박진석(전기공학부 19학번)
- 김재원(전기공학부 19학번)
- 김용진(전기공학부 18학번)

