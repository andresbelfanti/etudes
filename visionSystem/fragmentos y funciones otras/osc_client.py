from pythonosc.udp_client import SimpleUDPClient

ip = "192.168.0.31"
port = 12000
ene = [1,2,3,4,5,6,7,8]

client = SimpleUDPClient(ip, port)  # Create client
client.send_message("/some/address", ene)  # Send message with int, float and string