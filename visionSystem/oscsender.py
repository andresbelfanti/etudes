import time
from pythonosc.udp_client import SimpleUDPClient

def oscSender():
   print("oscSender")
 
   client.send_message("/prueba", "1")  # Send message with int, float and string


ip = "127.0.0.1"
port  = 58000
client = SimpleUDPClient(ip, port)  # Create client  

while(1):
   oscSender()
   time.sleep(0.5)
