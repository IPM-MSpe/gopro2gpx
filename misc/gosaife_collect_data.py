import os
import cv2
import json
import yaml
import rospy
import pyproj
import socket
import struct
import numpy as np
from datetime import datetime
# from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


# Main Parameters
host = '192.168.0.10'
port = 50000
min_dist = 0.5 # in meter
min_time = 30 # in seconds
output_folder = "output/"

# Preparations for MUM mini cameras
calib_file_path = 'outputs/params/Cam_{}_calib_Params.yaml'
chunk = 1310780
chunk_size = chunk*1
sockets = []
for i in range(3):
    sockets.append(socket.socket())
    sockets[i].connect((host, port + i))

# GPS Parameters
last_point = [0, 0]
last_time = datetime.now()
start_time = last_time
P = pyproj.Proj(proj='utm', zone=32, ellps='WGS84', preserve_units=True)
G = pyproj.Geod(ellps='WGS84')

# Misc Parameters
counter = 0
cam_strings = ["fl", "fr", "rc"]
json_template = {
    "city": "freiburg",
    "gps_lat": 0.0,
    "gps_lon": 0.0,
    "speed": 0.0,
    "date_time": "",
}

# Prep output folder
if not os.path.isdir(output_folder):
    try:
        os.mkdir(output_folder)
        print("Output folder created: " + output_folder)
    except Exception as e:
        print(e)
        exit(0)


def send_msg(sock, msg):
    # Prefix each message with a 4-byte length (network byte order)
    msg = struct.pack('>I', len(msg)) + msg
    sock.sendall(msg)


def recv_msg(sock):
    # Read message length and unpack it into an integer
    raw_msglen = recvall(sock, 4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    # Read the message data
    # data = recvall(sock, msglen)
    return recvall(sock, msglen)


def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data


def get_cam_stream(data):
    data_ = data.copy()
    height = 1024
    width = 1280
    image_size = height*width
    # if i == 0:
    offset_ = 56
    # else:
    #     offset_ = int(56+image_size+4)*i
    bin_image = np.frombuffer(bytes(data_), dtype=np.uint8, offset=offset_, count=image_size)
    bin_image.shape = (height, width)
    bin_image = cv2.cvtColor(bin_image, cv2.COLOR_BayerBG2RGB)
    bin_image = cv2.cvtColor(bin_image, cv2.COLOR_RGB2BGR)
    return bin_image


def check_version_payload(data):
    header_payload = np.frombuffer(bytes(data), dtype=np.uint32, offset=0, count=4)
    version = header_payload[1]
    payload = header_payload[-1]
    return version, payload


def cam_id_image_timestamp(data):
    image_header = np.frombuffer(bytes(data), dtype=np.uint16, offset=16, count=20)
    start_time = image_header[6] + (image_header[5] * 1e5 + image_header[5]) / 2 ** 32
    end_time = image_header[14] + (image_header[13] * 1e5 + image_header[12]) / 2 ** 32
    timestamp = (start_time + end_time) * 0.5
    cam_id = image_header[16]
    return cam_id, timestamp


def get_k_d(cam_id):
    calib_path = open(calib_file_path.format(cam_id))
    calib = yaml.safe_load(calib_path)
    k = np.array(calib['camera_matrix'])
    d = np.array(calib['dist_coeff'])
    return k, d


def image_correction(bin_image, cam_id):
    dim = bin_image.shape[:2]
    k, d = get_k_d(cam_id)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), k, dim, cv2.CV_16SC2)
    bin_image = cv2.remap(bin_image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return bin_image
    

def image_stream(sock_data):
    version, payload = check_version_payload(sock_data)
    if version == 2 and payload == 1310760:
        bin_image = get_cam_stream(sock_data)
        cam_id, _ = cam_id_image_timestamp(sock_data)
        # bin_image = image_correction(bin_image, cam_id)
        return bin_image
    else:
        return None


def xy_to_latlon(x, y):
    # Transform x, y utm coordinates to lat, lon format
    global P
    return P(x, y, inverse=True)


def calc_distance(x0, y0, x1, y1):
    # Calculate distance between two points
    return np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)


def gps_callback(data):
    # Callback function, called when new gps data arrives
    global sockets, chunk_size, min_dist, last_point, last_time, counter
    x = data.pose.position.x
    y = data.pose.position.y
    point = [x, y]

    t = datetime.now()
    delta_t = (t - last_time).total_seconds()
    distance = calc_distance(last_point[0], last_point[1], point[0], point[1])
    if distance > min_dist or delta_t > min_time:
        json_data = json_template
        lat, lon = xy_to_latlon(x, y)
        json_data["gps_lat"] = lat
        json_data["gps_lon"] = lon
        json_data["date_time"] = t.strftime("%Y/%m/%d, %H:%M:%S")
        json_data["speed"] = distance / delta_t * 3.6
        for j, s in enumerate(sockets):
            cam_data = recvall(s, chunk_size)
            image = image_stream(cam_data)
            filename = start_time.strftime("%d%m%Y_%H%M_") + str(counter).zfill(8) + "_" + cam_strings[j] + ".jpg"
            json_data["img_filename_" + cam_strings[j]] = os.path.join(os.path.split(output_folder)[1], "images", filename)
            cv2.imwrite(os.path.join(output_folder, "images", filename), image)

        json_object = json.dumps(json_data, indent=4)
        filename = start_time.strftime("%d%m%Y_%H%M_") + str(counter).zfill(8) + ".json"
        with open(os.path.join(output_folder, "json", filename), "w") as outfile:
            outfile.write(json_object)

        last_time = t
        last_point = point
        counter += 1


def main():
    # Main Function subscribes to *current pose* ros topic
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/current_pose", PoseStamped, gps_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
