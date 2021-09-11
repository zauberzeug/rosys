#!/usr/bin/env python3

from flask import Flask, send_from_directory
from glob import glob
import os


app = Flask(__name__)
camera_bins = os.path.expanduser('~/z_cam_binaries')


def get_binary():
    bins = glob(f'{camera_bins}/z_cam_*bin')
    print("bins:", bins)
    return os.path.basename(bins[-1])


@app.route('/ota/hash', methods=['GET'])
def ota_hash():
    try:
        return get_binary().replace('z_cam_', '').replace('.bin', '')
    except Exception as e:
        print(e)
        return "No camera binary available", 503


@app.route('/ota/binary', methods=['GET'])
def ota_binary():
    try:
        return send_from_directory(camera_bins, get_binary(), as_attachment=True)
    except Exception as e:
        print(e)
        return "No camera binary available", 503


if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0", port=5000, threaded=True)
