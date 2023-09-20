from openvino.runtime import Core

ie = Core()
devices = ie.available_devices

for device in devices:
    device_name = ie.get_property(device,'FULL_DEVICE_NAME')
    print(device_name)