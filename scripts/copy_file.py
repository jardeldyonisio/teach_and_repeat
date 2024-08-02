#!/usr/bin/env python3

import os
import shutil

def copy_file(source_path, target_folder_path):
    filename = os.path.basename(source_path)
    target_path = os.path.join(target_folder_path, filename)
    shutil.copy(source_path, target_path)