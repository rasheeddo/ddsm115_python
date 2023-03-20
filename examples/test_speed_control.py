from ddsm115 import ddsm115
import time

drive = ddsm115.MotorControl()

drive.set_drive_mode(_id=1, _mode=2)
drive.set_drive_mode(_id=2, _mode=2)

rpm_list = [10, 20, 30, 40, 50]
rpm_list2 = [50, 40, 30, 20, 10]
i = 0
last_rpm_change_stamp = time.time()
last_get_fb_stamp = time.time()

while True:

	try:

		if (time.time() - last_rpm_change_stamp) > 2.0:
			drive.send_rpm(_id=1, rpm=rpm_list[i])
			drive.send_rpm(_id=2, rpm=rpm_list2[i])

			i += 1

			if i == len(rpm_list):
				i = 0

			last_rpm_change_stamp = time.time()

		if (time.time() - last_get_fb_stamp) > 0.05:
			rpm_L, cur_L = drive.get_motor_feedback(_id=1)
			rpm_R, cur_R = drive.get_motor_feedback(_id=2)

			print(f"rpm_L: {rpm_L} rpm_R: {rpm_R} cur_L: {cur_L:.2f} cur_R: {cur_R:.2f}")

			last_get_fb_stamp = time.time()

	except KeyboardInterrupt:
		drive.send_rpm(_id=1, rpm=0)
		drive.send_rpm(_id=2, rpm=0)
		drive.close()
		break
