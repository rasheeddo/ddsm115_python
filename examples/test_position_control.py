from ddsm115 import ddsm115
import time

drive = ddsm115.MotorControl()

drive.set_drive_mode(_id=1, _mode=3)

deg_list = [90, 180, 270, 0]
last_deg_change_stamp = time.time()
last_get_fb_stamp = time.time()
i = 0
while True:

	if (time.time() - last_deg_change_stamp) > 3.0:
		print("deg {}".format(deg_list[i]))
		drive.send_degree(_id=1, deg=deg_list[i])

		i += 1

		if i == len(deg_list):
			i = 0

		last_deg_change_stamp = time.time()

	# if (time.time() - last_get_fb_stamp) > 0.05:
	# 	rpm_L, cur_L = drive.get_motor_feedback(_id=1)
	# 	# print(f"rpm_L: {rpm_L} rpm_R: {rpm_R} cur_L: {cur_L:.2f} cur_R: {cur_R:.2f}")
	# 	print(f"rpm_L: {rpm_L} cur_L: {cur_L:.2f}")

	# 	last_get_fb_stamp = time.time()