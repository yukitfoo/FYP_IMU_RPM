import socket
import time

host, port = "127.0.0.1", 25001
# data = "1,2,3"

# SOCK_STREAM means TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


# try:
#     # Connect to the server and send the data
#     # sock.connect((host, port))
#     sock.sendall(data.encode("utf-8"))
#     response = sock.recv(1024).decode("utf-8")
#     print (response)
def con():
    sock.connect((host, port))
    x = 10
    y = 20
    z = 30
    while True:
        data = f"{x},{y},{z}"
        sock.sendall(data.encode("utf-8"))
        response = sock.recv(1024).decode("utf-8")
        print(response)
        x += 1
        y += 1
        z += 1
        if x == 360:
            break
        time.sleep(0.01)
            
    sock.close()

con()



