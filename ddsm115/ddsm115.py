import serial
import serial.rs485
import struct
import crcmod.predefined
import numpy as np
import time

def print_info(text):
	print("DDSM115_INFO | {}".format(text))

def print_warning(text):
	print("DDSM115_WARNING | {}".format(text))

class MotorControl:

	def __init__(self, device="/dev/ttyUSB0"):

		# self.ser = serial.Serial(device, 115200)
		self.ser = serial.rs485.RS485(device, 115200, timeout=0)
		self.ser.rs485_mode = serial.rs485.RS485Settings()
		self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8-maxim')
		self.str_10bytes = ">BBBBBBBBBB"
		self.str_9bytes = ">BBBBBBBBB"

		self.prev_fb_rpm = [0,0,0,0]
		self.prev_fb_cur = [0,0,0,0]

	def close(self):
		self.ser.close()

	######################
	### Math Functions ###
	######################
	def Int16ToBytesArray(self, data: int):
		byte1 = (data & 0xFF00) >> 8
		byte2 = (data & 0x00FF)
		return [byte1, byte2]

	def TwoBytesTo16Int(self, high_byte: int, lo_byte: int):
		int16 = ((high_byte & 0xFF)) << 8 | (lo_byte & 0xFF)
		return np.int16(int16).item()

	def map(self, val, in_min, in_max, out_min, out_max):
		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def crc_attach(self, data_bytes: bytes):
		crc_int = self.crc8(data_bytes)
		data_bytesarray = bytearray(data_bytes)
		data_bytesarray.append(crc_int)
		full_cmd = bytes(data_bytesarray)
		
		return full_cmd

	def currentRawToCurrentAmp(self, cur_raw: int):
		return self.map(cur_raw, -32767, 32767, -8.0, 8.0)

	######################
	### Read/Write cmd ###
	######################
	def set_id(self, _id: int):
		"""
	 	connect only 1 motor, and call this function to set the ID of that motor
		"""

		SET_ID = struct.pack(self.str_10bytes, 0xAA, 0x55, 0x53, _id, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE)
		for i in range(5):
			self.ser.write(SET_ID)

	def send_rpm(self, _id: int, rpm):

		rpm = int(rpm)
		rpm_ints = self.Int16ToBytesArray(rpm)
		cmd_bytes = struct.pack(self.str_9bytes, _id, 0x64, rpm_ints[0], rpm_ints[1], 0x00, 0x00, 0x00, 0x00, 0x00)
		cmd_bytes = self.crc_attach(cmd_bytes)

		while not self.ser.writable():
			print_warning("send_rpm not writable")
			pass
		self.ser.write(cmd_bytes)

		_,_,_ = self.read_reply(_id)
		# res = self.ser.read_until(size=10)
		# print(cmd_bytes)
		# print_info(res)


	def send_degree(self, _id: int, deg):
		"""
		Args:
		- deg: in degrees

		Absolute angle position control.
		
		"""

		raw = int(self.map(deg, 0, 360, 0, 32767))

		deg_ints = self.Int16ToBytesArray(raw)
		cmd_bytes = struct.pack(self.str_9bytes, _id, 0x64, deg_ints[0], deg_ints[1], 0x00, 0x00, 0x00, 0x00, 0x00)
		cmd_bytes = self.crc_attach(cmd_bytes)

		self.ser.write(cmd_bytes)
		_,_,_ = self.read_reply(_id)
		# res = self.ser.read_until(size=10)
		# print(cmd_bytes)

	def set_brake(self, _id: int):

		cmd_bytes = struct.pack(self.str_9bytes, _id, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00)
		cmd_bytes = self.crc_attach(cmd_bytes)
		self.ser.write(cmd_bytes)
		res = self.ser.read_until(size=10)

	def set_drive_mode(self, _id: int, _mode: int):
		"""
		_mode: 0x01 current (torque), 0x02 velocity, 0x03 position
		"""

		if _mode == 1:
			print_info(f"Set {_id} as current (torque) mode")
		elif _mode == 2:
			print_info(f"Set {_id} as velocity mode")
		elif _mode == 3:
			print_info(f"Set {_id} as position mode")
		else:
			print_info(f"Error {_mode} is unknown")

		cmd_bytes = struct.pack(self.str_10bytes, _id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, _mode)
		self.ser.write(cmd_bytes)

	def get_motor_id(self):

		ID_QUE = struct.pack(self.str_10bytes, 0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE)
		self.ser.write(ID_QUE)
		data = self.ser.read_until(size=10)
		# print(data)
		print_info(f"ID: {data[0]}")
		print_info(f"Mode: {data[1]}")
		print_info(f"Error: {data[8]}")

	def get_motor_feedback(self, _id: int):

		fb_req_cmd = struct.pack(self.str_9bytes, _id, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
		fb_req_cmd = self.crc_attach(fb_req_cmd)
		while not self.ser.writable():
			print_warning("get_motor_feedback not writable")
			pass
		self.ser.write(fb_req_cmd)

		fb_rpm, fb_cur, error = self.read_reply(_id)
		if error != 0:
			sensor_error = error & 0b00000001
			over_current_error = error & 0b00000010
			phase_over_error = error & 0b00000100
			stall_error = error & 0b00001000
			troubleshoot_error = error & 0b00001000
			print_warning(f"error {error}")
			print_warning(f"sens_err: {sensor_error} phase_err: {phase_over_error} stall_err: {stall_error} trbs_err: {troubleshoot_error}")

		return fb_rpm, fb_cur
	
	def read_reply(self, _id, timeout=0.01):
		"""
		read a reply immediately after send write command.
		This read_reply is like a circular buffer or ring buffer concept,
		the read byte will be checked the first index as ID or not, second index as mode or not,
		if the head bytes are correctm then data field wil be stored in array. 
		And finally the index 10 is crc will be checked to confirm, the data is completely/correctly sent. 

		timout is an amount of time to ignore reading, because incoming bytes sometime it's not correct.
		So we repeatedly receive and check index. If finally the timeout is exceed, then we use previous value.
		"""

		got_reply = False
		ring_buffer = bytearray()
		start_time = time.time()
		while not got_reply:

			try:
				res = self.ser.read()
			except serial.serialutil.SerialException as e:
				print_warning(e)

			if len(res) != 0:
				## at first ring_buffer is empty, first byte is ID
				if (len(ring_buffer) == 0) and ((res == _id.to_bytes(1, 'big'))): 
					ring_buffer.append(int.from_bytes(res, 'big'))

				## 2nd byte is mode value, could be 0x01, 0x02, 0x03
				elif (len(ring_buffer) == 1) and ((res == b'\x02')):
					ring_buffer.append(int.from_bytes(res, 'big'))

				## size of ring_buffer not 10, and still some place to fill
				elif (len(ring_buffer) != 10) and (len(ring_buffer) >= 2):
					ring_buffer.append(int.from_bytes(res, 'big'))

					## check first if total size of ring_buffer is 10 or not
					if (len(ring_buffer) == 10):
						crc_value = ring_buffer[-1]
						raw_non_crc_bytes = bytes(ring_buffer[:-1])
						crc_check = self.crc8(raw_non_crc_bytes)

						# print(hex(ring_buffer[0]), hex(ring_buffer[1]), hex(ring_buffer[2]), hex(ring_buffer[3]), hex(ring_buffer[4]), hex(ring_buffer[5]),\
						#  hex(ring_buffer[6]), hex(ring_buffer[7]), hex(ring_buffer[8]), hex(ring_buffer[9]))

						if crc_value == crc_check:
							ID = ring_buffer[0]
							mode = ring_buffer[1]
							cur_hi = ring_buffer[2]
							cur_lo = ring_buffer[3]
							rpm_hi = ring_buffer[4]
							rpm_lo = ring_buffer[5]
							error = ring_buffer[8]
							fb_cur = self.currentRawToCurrentAmp(self.TwoBytesTo16Int(cur_hi, cur_lo))
							fb_rpm = self.TwoBytesTo16Int(rpm_hi, rpm_lo)
							# print("ID: {} mode: {} cur: {} rpm: {} error: {}".format(\
							# 	ID, mode, cur_fb, rpm_fb, error))

							self.prev_fb_rpm[_id-1] = fb_rpm
							self.prev_fb_cur[_id-1] = fb_cur

							got_reply = True

						else:
							print_warning("crc error")
							ring_buffer = bytearray()

				## reset ring_buffer
				else:
					ring_buffer = bytearray()
					# print_warning("reset ring_buffer")
			
			else:
				ring_buffer = bytearray()

			period = time.time() - start_time
			if period > timeout:
				got_reply = True
				# print_warning("over timeout")
				ID = _id
				mode = 0
				fb_rpm = self.prev_fb_rpm[_id-1]
				fb_cur = self.prev_fb_cur[_id-1]
				error = 0
				break

		return fb_rpm, fb_cur, error


if __name__ == "__main__":

	a = MotorControl()

	a.set_drive_mode(1, 2)
	a.send_rpm(1, 30)


	# while True:
	# 	a.get_motor_feedback(1)
	# 	time.sleep(0.1)

	# a.set_drive_mode(1, 3)
	# a.send_degree(1, 90)

	# a.get_motor_id()

	# a.set_id(2)