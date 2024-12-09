from socket import * 
import pickle, cv2, sys

"""
Program broadcasts live video for robot. Uses a file named clients.txt within
the same directory to obtain the IPs for sending video stream to.

Add a client to clients.txt file where each line has the following format:
    [hostname] [port number]
""" 

def main():
    clients = getClients()

    #Setup UDP socket
    host_video_UDP_socket = socket(AF_INET, SOCK_DGRAM)
    host_video_UDP_socket.setsockopt(SOL_SOCKET, SO_SNDBUF, 1000000)
   
    print("Ready for live video stream")

    #Setup camera
    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)

    while cap.isOpened():
        _, img = cap.read()
        _, buffer = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 30])

        serial_buffer = pickle.dumps(buffer)

        for client in clients:
            host_video_UDP_socket.sendto((serial_buffer),
                (client[0],client[1]))

        if cv2.waitKey(5) & 0xFF == 113:
            break

    cv2.destroyAllWindows()
    cap.release()

def getClients() -> list:
    """
    Extract hosts' ip & port pairs from text file.

    Returns:
        list: Contains tuples of hostnames (str) & port numbers (int).
    
    Raises:
        FileNotFoundError: If the file doesn't exist.
    """
    clients = []

    try:
        with open("clients.txt", "r") as fil:
            for line in fil: 
                pair = line.split()
                namePortPair = (pair[0],int(pair[1]))
                clients.append(namePortPair)

    except FileNotFoundError:
        raise FileNotFoundError("Clients file not found.")

    return clients



if __name__ == "__main__":
    main()