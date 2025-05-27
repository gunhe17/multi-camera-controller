### 배경

촬영 장치가 최고의 성능을 발휘할 수 있는 해상도, 프레임레이트, 인코딩 포맷 등을 파악하고, 실제 촬영 환경에서의 안정성과 효율성을 검증하기 위해 구현되었다.

### 실험 설계

Windows 환경에서 FFmpeg와 Media Foundation API를 이용하여 실험을 진행한다.

### 실행 및 결과

***1st***

기기 조회
```
ffmpeg -list_devices true -f dshow -i dummy
```

<br>

result
<br>
"HD Pro Webcam C920" (video)

<br>

***2nd***

기기 지원 촬영 해상도
```
ffmpeg -f dshow -list_options true -i video="HD Pro Webcam C920"
```

<br>

result

YUY2 (yuyv422)
| 해상도        | 최대 프레임레이트 |
| ---------- | --------- |
| 1280x720   | **10fps** |
| 1024x576   | 15fps     |
| 960x720    | 15fps     |
| 800x600    | 24fps     |
| 640x480 이하 | 30fps     |

<br>

MJPG
| 해상도        | 최대 프레임레이트 |
| ---------- | --------- |
| 1280x720   | **30fps** |
| 1920x1080  | **30fps** |
| 1600x896   | **30fps** |
| 640x480 이하 | 30fps     |

<br>
1280x720과 30fps를 만족하려면 MJPG를 선택해야만한다.

<br>

***3rd***

frame 간격 측정
```
# 'out.txt' override 시 오류 발생 가능

ffmpeg -f dshow -video_size 1280x720 -framerate 30 -i video="HD Pro Webcam C920" -vframes 3000 -f framehash -hash md5 lab\lab1\result\3rd_result.txt
```

<br>

*결과 file의 컬럼에 대한 설명이다.
| 컬럼       | 의미                                 |
| -------- | ---------------------------------- |
| stream#  | 스트림 번호                             |
| dts      | 디코딩 타임스탬프 (Decoding Time Stamp)    |
| pts      | 표시 타임스탬프 (Presentation Time Stamp) |
| duration | 프레임 지속 시간 (타임베이스 기준 단위)            |
| size     | 프레임 크기 (바이트 단위)                    |
| hash     | 프레임의 해시값 (MD5 등)                   |

<br>

result
<br>
결과를 통해 알 수 있듯이 ffmpeg를 통해 측정한 카메라는 잘 작동됨을 알 수 있다. 다만, 프레임 손실에 대한 부분을 살펴봐야 할 것 같다.

<br>

***4th***

ffmpeg와는 별개로 media foundation은 어떤 촬영 해상도를 지원하는지 확인해본 후, ffmpeg와 공통된 촬영 해상도 중 가장 적합한 것을 고르는 작업을 수행한다.

build
```
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil lab\lab1\4th_test.cpp /Fe:lab\lab1\bin\4th_test.exe ^
    mfplat.lib mfreadwrite.lib mf.lib mfuuid.lib ole32.lib
```

run
```
lab\lab1\bin\4th_test.exe
```

<br>

result
| 항목             | Media Type #293                          | Media Type #294                          |
| --------------- | ---------------------------------------- | ---------------------------------------- |
| **Resolution**  | 1280x720                                 | 1280x720                                 |
| **Frame Rate**  | 30 fps                                   | 30 fps                                   |
| **SubType**     | NV12                                     | MJPG                                     |
| **Interlace**   | 2 (Progressive)                          | 2 (Progressive)                          |
| **Sample Size** | (없음)                                    | 2,764,800 bytes                          |

window의 direct show를 기반으로 동작하는 ffmpeg는 NV12를 조회할 수 없다. NV12와 MJPG 중 어떤 것의 효율이 더 높은지에 대한 시험이 필요하다.


***+@***

최적의 촬영 스크립트

```
ffmpeg -f dshow -video_size 1280x720 -framerate 30 -i video="HD Pro Webcam C920" -vframes 90 -f null -
ffmpeg -f dshow -video_size 1280x720 -framerate 30 -i video="HD Pro Webcam C920" -vframes 3000 -c copy lab\lab1\result\raw_capture.avi
```

.mp4를 output format으로 설정하는 것보다, 인코딩 작업이 필요 없는 .avi를 output format으로 설정하는 편이 낫다.