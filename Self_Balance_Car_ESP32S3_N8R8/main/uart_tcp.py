import socket

ESP32_IP = "192.168.50.24"
PORT = 8080

def connect_to_server(ip, port):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((ip, port))
    print(f"Connected to {ip}:{port}")
    return client

while True:
    try:
        client = connect_to_server(ESP32_IP, PORT)
        while True:
            data = client.recv(1024)
            if not data:
                print("\nConnection lost.")
                break
            print(data.decode('utf-8'), end="")
    except (ConnectionError, OSError):
        print("\nConnection error occurred.")
    except KeyboardInterrupt:
        print("\nConnection closed by user.")
        break
    finally:
        try:
            client.close()
        except NameError:
            pass

    user_input = input("Do you want to reconnect? (Y/N): ").strip().upper()
    if user_input != "Y":
        print("Exiting program.")
        break
