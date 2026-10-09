from motor_test import MotorLink

link = MotorLink("COM3", sample_ms=5)

try:
    acceleration, deceleration = link.read_motion_limits()

    print(f"最大速度: {link.max_velocity} μm/s")
    print(f"加速度:   {acceleration} μm/s²")
    print(f"减速度:   {deceleration} μm/s²")
finally:
    link.close()