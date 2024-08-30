from video_stitcher import VideoStitcher

def main():
    input_videos = ["../example/2024-07-24_TiR360_Avata_DRZ_InnenAussen/VID_20220829_135321_00_004.insv","../example/2024-07-24_TiR360_Avata_DRZ_InnenAussen/VID_20220829_135321_10_004.insv"]
    report_id = "12345"
    output_video = "../example/2024-07-24_TiR360_Avata_DRZ_InnenAussen/output.avi"
    stitcher_calibration_path = "../config/calibration.json"
    thread = VideoStitcher("1", "2", report_id, stitcher_calibration_path, input_videos, output_video)
    thread.start()

if __name__ == "__main__":
    main()
