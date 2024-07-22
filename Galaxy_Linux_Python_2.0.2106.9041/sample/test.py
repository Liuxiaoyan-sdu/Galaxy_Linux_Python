import gxipy as gx
import sys

device_manager = gx.DeviceManager()
dev_num, dev_info_list = device_manager.update_device_list()
if dev_num == 0:
    sys.exit(1)

str_sn = dev_info_list[0].get("sn")
cam = device_manager.open_device_by_sn(str_sn)