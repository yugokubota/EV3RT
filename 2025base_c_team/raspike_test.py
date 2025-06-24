import threading
import time
import sys
import libraspike_art_python as lib
from libraspike_art_python import pbio_port, pbio_color, hub_button, sound, pup_direction

RASPIKE_COM_NAME = '/dev/USB_SPIKE'

def receiver_task():
    while True:
        lib.raspike_prot_receive()

def ultrasonicsensor_test():
    us = lib.pup_ultrasonic_sensor_get_device(pbio_port.ID_F)
    time.sleep(1)
    while True:
        print('US: Distance = %d, presence =%d' %(
	        lib.pup_ultrasonic_sensor_distance(us),
	        lib.pup_ultrasonic_sensor_presence(us)
        ))
        time.sleep(1)

def motor_test():
    mot1 = lib.pup_motor_get_device(pbio_port.ID_A)
    mot2 = lib.pup_motor_get_device(pbio_port.ID_B)
    mot3 = lib.pup_motor_get_device(pbio_port.ID_C)
    err = lib.pup_motor_setup(mot1,pup_direction.CLOCKWISE,True)
    err = lib.pup_motor_setup(mot2,pup_direction.CLOCKWISE,True)
    err = lib.pup_motor_setup(mot3,pup_direction.CLOCKWISE,True)

    for power in range(-100, 101, 10):
        lib.pup_motor_set_power(mot1,power)
        lib.pup_motor_set_power(mot2,power)
        lib.pup_motor_set_power(mot3,power)
        time.sleep(1)
    
    lib.pup_motor_stop(mot2)
    lib.pup_motor_stop(mot3)

    lib.pup_motor_set_speed(mot1,400)
    time.sleep(1)
    lib.pup_motor_stop(mot1)
    time.sleep(1)
    lib.pup_motor_set_speed(mot1,-400)
    time.sleep(1)
    lib.pup_motor_brake(mot1)
    time.sleep(1)
    lib.pup_motor_set_speed(mot1,400)  
    time.sleep(1)
    lib.pup_motor_hold(mot1)
    time.sleep(1)

    for i in range(-400, 401, 40):
        lib.pup_motor_set_speed(mot1,i)
        print('mot1:power=%d speed=%d count=%d isStall=%d' %(
            lib.pup_motor_get_power(mot1),
	        lib.pup_motor_get_speed(mot1),
	        lib.pup_motor_get_count(mot1),
            lib.pup_motor_is_stalled(mot1)
        ))
        time.sleep(1)

    lib.pup_motor_stop(mot1)

def display_test():
    lib.hub_display_number(12)
    time.sleep(3)

    lib.hub_display_char('X')
    time.sleep(2)

    #lib.hub_display_text('Ras Pike!',1000,1000)
    lib.hub_display_text_scroll('ETRobocon20th',500)

def light_test():
    lib.hub_light_on_hsv(30, 100, 100)
    time.sleep(2)
    lib.hub_light_on_color(pbio_color.GREEN)
    time.sleep(2)
    lib.hub_light_off()

def speaker_test():
    lib.hub_speaker_set_volume(50)
    lib.hub_speaker_play_tone(sound.NOTE_C5,2000)
    lib.hub_speaker_play_tone(sound.NOTE_A5,sound.MANUAL_STOP)
    time.sleep(3)
    lib.hub_speaker_stop()
    lib.hub_light_on_color(pbio_color.GREEN)
    time.sleep(3)
    #lib.hub_system_shutdown()

def colorsensor_test():
    col = lib.pup_color_sensor_get_device(pbio_port.ID_E)

    for i in range(10):
        r, g, b = lib.pup_color_sensor_rgb(col)
        print(f'[RGB] r={r:>3} g={g:>3} b={b:>3}')
        h, s, v = lib.pup_color_sensor_hsv(col, True)
        print(f'[HSV] h={h:>3} s={s:>3} v={v:>3}')
        time.sleep(1)

def battery_test():
    cur = lib.hub_battery_get_current()
    vol = lib.hub_battery_get_voltage()
    print(f'[battery] cur={cur}, vol={vol}')

def imu_test():
    lib.hub_imu_init()
    time.sleep(1)

    for i in range(10):
        x, y, z = lib.hub_imu_get_angular_velocity()
        print(f'[IMU ang velo] x={x:>10.3f} y={y:>10.3f} z={z:>10.3f}')
        x, y, z = lib.hub_imu_get_acceleration() 
        print(f'[IMU accel]    x={x:>10.3f} y={y:>10.3f} z={z:>10.3f}')
        time.sleep(1)

def button_test():
    while not lib.hub_button_is_pressed(hub_button.CENTER):
        pass
    lib.hub_speaker_set_volume(50)
    lib.hub_speaker_play_tone(sound.NOTE_A5,sound.MANUAL_STOP)
    time.sleep(1)
    lib.hub_speaker_stop()
    while not lib.hub_button_is_pressed(hub_button.CENTER):
        pass
    lib.hub_speaker_play_tone(sound.NOTE_A6,sound.MANUAL_STOP)
    time.sleep(1)
    lib.hub_speaker_stop()

if __name__ == "__main__":
    desc = lib.raspike_open_usb_communication(RASPIKE_COM_NAME)
    if desc is None:
        print(f"Cannot Open desc name={RASPIKE_COM_NAME}")
        sys.exit(-1)
    
    lib.raspike_prot_init(desc)

    t1 = threading.Thread(target=receiver_task,daemon=True)
    t1.start()

    #ultrasonicsensor_test()
    #display_test()
    #light_test()
    #speaker_test()
    #motor_test()
    battery_test()
    imu_test()
    colorsensor_test()
    #button_test()
