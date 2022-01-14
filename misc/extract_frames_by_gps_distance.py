import os
import cv2
import csv
import json
import gpxpy
import argparse
import numpy as np
from tqdm import tqdm

parser = argparse.ArgumentParser(description='Extract frames from videos according to GPS locations.')

parser.add_argument('--gpx', action="store", required=True)
parser.add_argument('--vid', action="store", required=True)
parser.add_argument('--output', action="store", required=True)
parser.add_argument('--min_dist', action="store", default=25)
parser.add_argument('--fov', action="store", default=90.0, type=float)
parser.add_argument('--f_angle', action="store", default=45.0, type=float)
parser.add_argument('--city', action="store", default="freiburg")

args = parser.parse_args()

gpx_file_name = args.gpx
vid_file_name = args.vid
out_folder = args.output
rel_output_folder = os.path.split(args.output)[1]
out_folder_img = "images"
out_folder_json = "json"
min_dist = args.min_dist
min_start_time = 5
city = args.city
out_im_size = (2048, 1024)
out_cam_fov = args.fov
out_cam_angle = args.f_angle
smoothen_num_items = 5

# Init stuff
json_data = {"city": city,
             "gps_lat": 0,
             "gps_lon": 0,
             "speed": 0,
             "date_time": 0
             }
name_exts = ["_fc", "_fl", "_fr", "_rc"]
counter_processed = 0
cskippd = 0
start_time = 0
prev_time = 0
gpx_file = open(gpx_file_name, "r")
vid_file = cv2.VideoCapture(vid_file_name)
total_frames = vid_file.get(cv2.CAP_PROP_FRAME_COUNT)
print("Processing: " + gpx_file_name + " / " + vid_file_name)

# Prep folders and files
if not os.path.isdir(os.path.join(out_folder, out_folder_img)) or not os.path.isdir(os.path.join(out_folder, out_folder_json)):
    try:
        if not os.path.isdir(out_folder):
            os.mkdir(out_folder)
        os.mkdir(os.path.join(out_folder, out_folder_img))
        os.mkdir(os.path.join(out_folder, out_folder_json))
        print("Output folder created: " + out_folder)
    except Exception as e:
        print(e)
        exit(0)
csv_file = open(os.path.join(out_folder, os.path.splitext(os.path.split(vid_file_name)[1])[0] + ".csv"), 'w', encoding='UTF8', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["ID", "Latitude", "Longitude", "Speed", "Image"])

# Parse GPX file for location and time stamps
plot_t = []
plot_s = []
t_orig = []
gpx = gpxpy.parse(gpx_file)
counter_index = 0
for track in gpx.tracks:
    # Merge segments if multiple exist
    if len(track.segments) > 1:
        for i in range(len(track.segments)):
            track.segments[0].join(track.segments[i])
    segment = track.segments[0]

    # Calculate fractions of seconds for precise extraction of frames
    same_time_counter = 1
    for c, point in enumerate(segment.points):
        if start_time == 0:
            start_time = point.time     # Store start time of the video
            prev_time = point.time
            point.speed = 0
        if point.time == prev_time:
            same_time_counter += 1
        else:
            for i in range(same_time_counter):
                fract = i / same_time_counter * 1000000
                segment.points[c - same_time_counter + i].time = segment.points[c - same_time_counter + i].time.replace(microsecond=int(fract))
            same_time_counter = 1
            prev_time = point.time
        t_orig.append((point.time - start_time).total_seconds())
        point.dgps_id = counter_index
        counter_index += 1

    # Set fractions of seconds for the last items in list (not processed during last loop)
    for i in range(same_time_counter):
        fract = i / same_time_counter * 1000000
        segment.points[-same_time_counter + i].time = segment.points[- same_time_counter + i].time.replace(microsecond=int(fract))

    # Calculate and set speed
    for c, point in enumerate(segment.points):
        prev_idx = max((c - smoothen_num_items, 0))
        if c == 0:
            point.speed = 0
        else:
            point.speed = point.speed_between(segment.points[prev_idx]) * 3.6

    # Reduce points down according to minimal distance between points
    reduced_segment = segment.clone()
    reduced_segment.reduce_points(min_dist)

    # Iterate through points and extract video frames
    for c, point in enumerate(tqdm(segment.points)):
        out_file_name_json = ""
        new_json = json_data
        frame_time_in_ms = int((point.time - start_time).total_seconds() * 1000)

        # Determine if current point is in set of reduced points
        in_reduced_segment = False
        for point_r in reduced_segment.points:
            if point.dgps_id == point_r.dgps_id:
                in_reduced_segment = True
                break

        # Throw away first couple of seconds
        if frame_time_in_ms > min_start_time * 1000:

            # Write JSON files and export images only for reduced set
            if in_reduced_segment:
                output_file_name = city \
                                   + "_" + str(point.time.year) + str(point.time.month).zfill(2) + str(point.time.day).zfill(2) \
                                   + "_" + os.path.splitext(os.path.split(vid_file_name)[1])[0] \
                                   + "_" + str(counter_processed).zfill(10)

                # Check if frame is found in video
                vid_file.set(cv2.CAP_PROP_POS_MSEC, frame_time_in_ms)
                frame_found, frame = vid_file.read()
                if frame_found:
                    images = []
                    px_per_deg = (float(frame.shape[0]) / 360.0, float(frame.shape[1]) / 360.0)
                    half_im_size = (int(out_cam_fov / 2 * px_per_deg[0]), int(out_cam_fov / 2 * px_per_deg[1]))

                    # Extract image for central front cam
                    center = (int(frame.shape[0] / 2), int(frame.shape[1] / 2))
                    images.append(frame[center[0]-half_im_size[0]:center[0]+half_im_size[0], center[1]-half_im_size[1]:center[1]+half_im_size[1], :])

                    # Extract image for front cam left
                    center = (int(frame.shape[0] / 2), int((-out_cam_angle + 180.0) * px_per_deg[1]))
                    images.append(frame[center[0]-half_im_size[0]:center[0]+half_im_size[0], center[1]-half_im_size[1]:center[1]+half_im_size[1], :])

                    # Extract image for front cam right
                    center = (int(frame.shape[0] / 2), int((out_cam_angle + 180.0) * px_per_deg[1]))
                    images.append(frame[center[0]-half_im_size[0]:center[0]+half_im_size[0], center[1]-half_im_size[1]:center[1]+half_im_size[1], :])

                    # Extract image for rear cam
                    image_l = frame[center[0]-half_im_size[0]:center[0]+half_im_size[0], 0:half_im_size[1],:]
                    image_r = frame[center[0]-half_im_size[0]:center[0]+half_im_size[0], -half_im_size[1]-1:-1,:]
                    images.append(np.concatenate((image_r, image_l), axis=1))

                    # Write files
                    for i, image in enumerate(images):
                        out_file_name_image = os.path.join(out_folder, out_folder_img, output_file_name + name_exts[i] + ".jpg")
                        new_json["img_filename" + name_exts[i]] = os.path.join(rel_output_folder, out_folder_img, output_file_name + name_exts[i] + ".jpg")

                        image_scaled = cv2.resize(image, out_im_size, interpolation=cv2.INTER_AREA)
                        outimg_file_name_disk = os.path.splitext(out_file_name_image)[0] + name_exts[i] + os.path.splitext(out_file_name_image)[1]
                        cv2.imwrite(out_file_name_image, image_scaled)

                    # Serializing json data
                    new_json["gps_lat"] = point.latitude
                    new_json["gps_lon"] = point.longitude
                    new_json["speed"] = point.speed
                    new_json["date_time"] = point.time.strftime("%Y/%m/%d, %H:%M:%S")
                    json_object = json.dumps(new_json, indent = 4)
                    out_file_name_json = os.path.join(out_folder, out_folder_json, output_file_name + ".json")
                    with open(out_file_name_json, "w") as outfile:
                        outfile.write(json_object)
                    counter_processed += 1
                else:
                    cskippd += 1
        else:
            cskippd += 1

        # Export every point as CSV data
        row = [point.dgps_id, point.latitude, point.longitude, point.speed, os.path.split(out_file_name_json)[1]]
        csv_writer.writerow(row)

    print("Extracted {0}/{1} frames for {2} datapoints (skipped {3}).".format(counter_processed, int(total_frames), segment.get_points_no(), cskippd))
csv_file.close()
