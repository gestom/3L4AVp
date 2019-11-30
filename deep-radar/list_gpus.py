from tensorflow.python.client import device_lib

devs = device_lib.list_local_devices()
devs = [x.name for x in devs if x.device_type == 'GPU']
print("List of available gpus:")
print(devs)
