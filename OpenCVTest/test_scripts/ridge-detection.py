from ridge_detection.lineDetector import LineDetector
from ridge_detection.params import Params,load_json
from ridge_detection.basicGeometry import reset_counter
from ridge_detection.helper import displayContours,save_to_disk
from argparse import ArgumentParser
from datetime import datetime
from PIL import Image
from  mrcfile import open as mrcfile_open


def run():
    start=datetime.now()
    parser = ArgumentParser("ridge detection parser tool")
    parser.add_argument(dest="config_filename",type=str, nargs='?',help="name of the config_file to use. Default value is 'example_config.json'")
    args=parser.parse_args()
    config_filename = args.config_filename if args.config_filename is not None else "example_config.json"
    json_data=load_json(config_filename)
    params = Params(config_filename)

    try:
        img=mrcfile_open(json_data["path_to_file"]).data
    except ValueError:
        img=Image.open(json_data["path_to_file"])

    detect = LineDetector(params=config_filename)
    result = detect.detectLines(img)
    resultJunction =detect.junctions
    out_img,img_only_lines = displayContours(params,result,resultJunction)      
    if params.get_saveOnFile() is True:
        save_to_disk(out_img,img_only_lines)

    print(" TOTAL EXECUTION TIME: " + str(datetime.now()-start))

if __name__ == "__main__":
    run()