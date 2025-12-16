import serial
import time
import Jetson.GPIO as GPIO

LED_PIN = 11
GPIO.setmode(GPIO.BOARD)  # BOARD 번호 체계 사용
GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.output(LED_PIN, GPIO.LOW)

# 시리얼 포트 설정 (ttyTHS1을 사용)
ser = serial.Serial('/dev/ttyTHS0', 1000000, timeout=1)  # 115200은 Baudrate

try :
    while True:
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(0.00001)
        ping_hex = b'\xff\xff\xfe\x01\x02\x01\x03\x01\xb9\x56'
        ser.write(ping_hex)
        time.sleep(0.00001)  # 2초 대기
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(0.0005)  # 2초 대기
        response = ser.read(ser.in_waiting or 1) 
        if response:
            print(f"Received: {response.hex().upper()}")
        else: print("no response")

except KeyboardInterrupt:
    # Ctrl+C로 종료 시
    print("프로그램 종료")
finally:
    # GPIO 정리
    GPIO.cleanup()
    ser.close()
    print("GPIO 설정이 정리되었습니다.")

