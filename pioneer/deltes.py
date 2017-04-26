with open('/home/jonathan/catkin_ws/src/drcs/pioneer/ros_control.py', 'rb+') as f:
    content = f.read()
    f.seek(0)
    f.write(content.replace(b'\r', b''))
    f.truncate()