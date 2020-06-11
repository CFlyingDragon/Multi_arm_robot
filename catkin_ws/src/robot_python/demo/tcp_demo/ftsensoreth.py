#***********************************************************************
#
# NRS-6-X Ethernet TCP/IP example.
#
# ----------------------------------------------------------------------
#
# Author: Thor Stork Stenvang <thor@nordbo-robotics.com>
# Date: 09-10-2018
#
# ----------------------------------------------------------------------
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
#***********************************************************************/
import struct
import socket

IP_ADDR	= '192.168.0.100';
PORT	= 2001;

CMD_TYPE_SENSOR_TRANSMIT 	= '07'
SENSOR_TRANSMIT_TYPE_START 	= '01'
SENSOR_TRANSMIT_TYPE_STOP 	= '00'

CMD_TYPE_SET_CURRENT_TARE 	= '15'
SET_CURRENT_TARE_TYPE_NEGATIVE	= '01'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	

def main():
	global s
	s.settimeout(2.0)
	s.connect((IP_ADDR, PORT))

	sendData = '03' + CMD_TYPE_SET_CURRENT_TARE + SET_CURRENT_TARE_TYPE_NEGATIVE
	sendData = bytearray.fromhex(sendData)
	s.send(sendData)
	recvData = recvMsg()

	sendData = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_START
	sendData = bytearray.fromhex(sendData)
	s.send(sendData)
	recvData = recvMsg()

	for i in range(10000):
		recvData = recvMsg()
		Fx = struct.unpack('!d', recvData[2:10])[0]
		Fy = struct.unpack('!d', recvData[10:18])[0]
		Fz = struct.unpack('!d', recvData[18:26])[0]
		Tx = struct.unpack('!d', recvData[26:34])[0]
		Ty = struct.unpack('!d', recvData[34:42])[0]
		Tz = struct.unpack('!d', recvData[42:50])[0]
		print(str(Fx) + " " + str(Fy) + " " + str(Fz) + " " + str(Tx) + " " + str(Ty) + " " + str(Tz))

	sendData = '03' + CMD_TYPE_SENSOR_TRANSMIT + SENSOR_TRANSMIT_TYPE_STOP
	sendData = bytearray.fromhex(sendData)
	s.send(sendData)
	recvData = recvMsg()

	#Wait until and ACK msg is send back for the stop command. 
	while recvData[0] != 3 and recvData[1] != CMD_TYPE_SENSOR_TRANSMIT:
		recvData = recvMsg()

	s.close()

def recvMsg():
	recvData = bytearray(s.recv(2))

	while len(recvData) < recvData[0] :
		recvData += bytearray(s.recv(recvData[0] - len(recvData)))

	printMsg(recvData)

	return recvData

def printMsg(msg):
	print("Msg len: " + str(msg[0]) + " Msg type: " + str(msg[1]) + "")
	
	dataStr = "DATA: "
	for i in range(msg[0] - 2):
		dataStr += str(msg[i + 2]) + " "

	print(dataStr)

if __name__ == "__main__":
	main()
