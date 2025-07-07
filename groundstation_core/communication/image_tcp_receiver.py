import socket
import rospy
import cv2
import pickle
import struct
import numpy as np
import os
from datetime import datetime
from sensor_msgs.msg import Image
import queue
import threading
# from vidgear.gears import WriteGear

class ImageProcessor:
    def __init__(self):
        self.image_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
        
        self.color_queue = queue.Queue()
        self.depth_queue = queue.Queue()
        # self.pub_thread = threading.Thread(target = self.publish_image, args=(self.color_queue,self.depth_queue))
        self.lock = 0
        
    def main(self):
        # 設定伺服器的IP地址和埠口
        server_ip = '192.168.12.1'  # 監聽所有網路介面的連線
        server_port = 10050  # 與影像傳送程式使用的目標端口一致

        # 建立TCP伺服器
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((server_ip, server_port))
        server_socket.listen(1)

        print("waiting for connect...")
        client_socket, address = server_socket.accept()
        print("connected：", address)

        data = b''  # 用於累積接收的影像資料

        payload_size = struct.calcsize("Q")
        count = 0
        
        s = datetime.strftime(datetime.now(),'%H%M%S') 

        # fourcc = cv2.VideoWriter_fourcc(*'XVID')          # 設定影片的格式為 MJPG
        # rgb_video_writer = cv2.VideoWriter(f'./data/{s}/rgb.avi', fourcc, 6.0, (640,  480))  # 產生空的影片
        # depth_video_writer = cv2.VideoWriter(f'./data/{s}/depth.avi', fourcc, 6.0, (640,  480),0)  # 產生空的影片

        # self.pub_thread.start()

        while True:
            while len(data) < payload_size:
                packet = client_socket.recv(4*1024)
                if not packet: break
                data+=packet
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q",packed_msg_size)[0]
            while len(data) < msg_size:
                data += client_socket.recv(4*1024)
            count += 1
            print('recieved', count)
            
            frame_data = data[:msg_size]
            data  = data[msg_size:]
            
            frame = pickle.loads(frame_data)
            frame  = cv2.imdecode(frame, cv2.IMREAD_UNCHANGED)
            print("frame dimension:", frame.ndim)
                

            if frame.ndim == 3:
                rgb_frame = frame.astype(np.uint8)
                print("RGB Frame Shape:", rgb_frame.shape)
                print("RGB Frame Type:", rgb_frame.dtype)
                print("RGB Frame Min Value:", np.min(rgb_frame))
                print("RGB Frame Max Value:", np.max(rgb_frame))
                # self.color_queue.put(rgb_frame)
                # self.publish_image()
                cv2.imshow("Receiving RGB...",rgb_frame)
                # rgb_video_writer.write(rgb_frame)       # 將取得的每一幀圖像寫入空的影片
                # cv2.waitKey(1)
            else:
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(frame, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow("Receiving Depth...",depth_colormap)
                
                # save_depth_frame = np.stack([frame % 256,frame % 256,frame // 256],axis=2).astype(np.uint8)
                # depth_colormap2 = cv2.applyColorMap(cv2.convertScaleAbs((save_depth_frame[:,:,0] + save_depth_frame[:,:,2]*256), alpha=0.03), cv2.COLORMAP_JET)
                # cv2.imshow("Receiving Depth3...",save_depth_frame)
                # cv2.imshow("Receiving Depth2...",depth_colormap2)
                # depth_video_writer.write((frame//256))

            

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # rgb_video_writer.release()
        # depth_video_writer.release()
    # def publish_image(self,color_queue,depth_queue):
    #     while True:
    #         print("+++++++++++++++++++++++++++++++++++++++++++++++++++")
    #         pub_color_image = color_queue.get()
    #         print("color image send",pub_color_image)

    #         # Publish the image message
    #         self.image_pub.publish(pub_color_image)

    #     # Spin ROS node
    #     # rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_publisher', anonymous=True)
    image_processor = ImageProcessor()
    image_processor.main()