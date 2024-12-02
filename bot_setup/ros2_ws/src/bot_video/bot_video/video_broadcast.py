from socket import * 
import pickle, cv2, sys

"""
Program broadcasts live video of robots floor feed. Before calling this
progam, pass host and port arguments for video server. 
""" 

def main():
    host = sys.argv[0]
    server_video_port = sys.argv[1]

    #Setup UDP socket
    client_video_UDP_socket = socket(AF_INET, SOCK_DGRAM)
    client_video_UDP_socket.setsockopt(SOL_SOCKET, SO_SNDBUF, 1000000)
   
    print("[video_thread] Ready for live video stream")

    #Setup camera
    cap = cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)

    while cap.isOpened():
        ret, img = cap.read()
        ret, buffer = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 30])

        serial_buffer = pickle.dumps(buffer)

        client_video_UDP_socket.sendto((serial_buffer),(host,server_video_port))

        if cv2.waitKey(5) & 0xFF == 113:
            break

    cv2.destroyAllWindows()
    cap.release()

if __name__ == "__main__":
    assert len(arg.v) == 2, "Two program arguments required, i.e. HOST & Port"
    main()