#!/usr/bin/env python3
from mp_test_ui.mp_test_widget import MpTestWidget

from rqt_gui_py.plugin import Plugin


class MpTest(Plugin):

    def __init__(self, context):
        super(MpTest, self).__init__(context)
        self.setObjectName('RQt example')
        self.widget = MpTestWidget(context.node)
        serial_number = context.serial_number()
        if serial_number > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + ' ({0})'.format(serial_number))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        print('Shutdown the RQt example.')
        self.widget.shutdown_widget()