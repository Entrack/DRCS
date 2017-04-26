import gtk
import sys
from gettext import gettext as _

from ros_control import ROSControl

VELOCITY = 0.5

class UICmd:
    """UI Class for robot control"""

    def __init__(self, topic):
        self.client = ROSControl(topic)

        window = gtk.Window()
        window.set_title(_('ROS UI Cmd'))
        window.resize(200, 200)
        window.connect('delete-event', self.stop)

        window.connect('key-press-event', self.key_press_callback)
        window.connect('key-release-event', self.key_release_callback)

        button_forward = gtk.Button(_('Forward'))
        button_forward.connect('pressed', self.command_forward)
        button_forward.connect('released', self.command_stop)

        button_backward = gtk.Button(_('Backward'))
        button_backward.connect('pressed', self.command_backward)
        button_backward.connect('released', self.command_stop)

        button_left = gtk.Button(_('Left'))
        button_left.connect('pressed', self.command_left)
        button_left.connect('released', self.command_stop)

        button_right = gtk.Button(_('Right'))
        button_right.connect('pressed', self.command_right)
        button_right.connect('released', self.command_stop)

        buttons_hbox = gtk.HBox()
        buttons_hbox.add(button_left)
        buttons_hbox.add(button_right)
        buttons_vbox = gtk.VBox()
        buttons_vbox.add(button_forward)
        buttons_vbox.add(buttons_hbox)
        buttons_vbox.add(button_backward)

        window.add(buttons_vbox)
        window.show_all()

        self.key_commands = {\
                65362: self.command_forward,
                65364: self.command_backward,
                65361: self.command_left,
                65363: self.command_right,
        }

    def key_press_callback(self, widget, event):
        keyval = event.keyval
        if keyval in self.key_commands.keys():
            self.key_commands[keyval]()

    def key_release_callback(self, widget, event):
        self.command_stop()

    def command_forward(self, *args):
        self.client.set_vel(VELOCITY, 0)
    def command_backward(self, *args):
        self.client.set_vel(-VELOCITY, 0)
    def command_left(self, *args):
        self.client.set_vel(0, -2 * VELOCITY)
    def command_right(self, *args):
        self.client.set_vel(0,  2 * VELOCITY)
    def command_stop(self, *args):
        self.client.set_vel(0, 0)

    def event_loop(self, *args):
        while gtk.events_pending():
            gtk.main_iteration(block=False)
        return True

    def start(self):
        self.client.main_loop = self.event_loop
        self.client.start(timeout=0.01)

    def stop(self, *args):
        sys.exit(0)


if __name__ == '__main__':
    UICmd('pioneer2dx').start()
