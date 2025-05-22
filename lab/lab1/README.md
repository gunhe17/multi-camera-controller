### 배경

선택한 촬영 기기가 적합한 기기인지 확인하는 작업의 필요성을 느꼈다. 선택한 촬영 기기가 적절한 frame 간격을 유지하며, 촬영할 수 있는 기기인지 시험하고자 구현하게 되었다.

### 실험 설계

FFMPEG cmd를 사용한다.

### 실행

기기 조회
```
ffmpeg -list_devices true -f dshow -i dummy
```

<br>

기기 지원 촬영 해상도
```
ffmpeg -f dshow -list_options true -i video="HD Pro Webcam C920"
```

<br>

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

frame 간격 측정
```
# 'out.txt' override 시 오류 발생 가능

ffmpeg -f dshow -video_size 1280x720 -framerate 30 -i video="HD Pro Webcam C920" -vframes 3000 -f framehash -hash md5 lab\lab1\result\1st_result.txt
```

### 결과

***1st***

*결과 file의 컬럼에 대한 설명이다.
| 컬럼       | 의미                                 |
| -------- | ---------------------------------- |
| stream#  | 스트림 번호                             |
| dts      | 디코딩 타임스탬프 (Decoding Time Stamp)    |
| pts      | 표시 타임스탬프 (Presentation Time Stamp) |
| duration | 프레임 지속 시간 (타임베이스 기준 단위)            |
| size     | 프레임 크기 (바이트 단위)                    |
| hash     | 프레임의 해시값 (MD5 등)                   |

결과를 통해 알 수 있듯이 ffmpeg를 통해 측정한 카메라는 잘 작동됨을 알 수 있다.