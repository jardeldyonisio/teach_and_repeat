#!/usr/bin/env python3

import os

from datetime import datetime

from copy_file import copy_file

def create_folder_with_datetime(folder_path):
    current_datetime = datetime.now()
    formatted_datetime = current_datetime.strftime("%d-%m-%Y_%H-%M-%S")
    folder_name = f"{formatted_datetime}"

    folder_path = os.path.join(folder_path, folder_name)

    os.makedirs(folder_path)

    return folder_path
